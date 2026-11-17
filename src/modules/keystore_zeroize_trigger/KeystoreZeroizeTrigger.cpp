/****************************************************************************
 *
 *   Copyright (c) 2026 Technology Innovation Institute. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file KeystoreZeroizeTrigger.cpp
 *
 * PROOF OF CONCEPT: hold a GPIO high for 5 s -> erase the NOR keystore -> reboot.
 * See KeystoreZeroizeTrigger.hpp.
 */

#include <board_config.h>
#include "KeystoreZeroizeTrigger.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/time.h>

#include <nuttx/mtd/mtd.h>

#include <keystore_backend_definitions.h>

#include <errno.h>
#include <string.h>

/* Provided by src/drivers/imx9_keystore (keystore_backend) and the board NOR driver. */
extern "C" {
	void keystore_init(void);
	void keystore_erase(void);
	struct mtd_dev_s *imx9_nor_device_get(void);
}

static constexpr char kKeystoreSignature[] = "nxp93_keystore";

KeystoreZeroizeTrigger::KeystoreZeroizeTrigger() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

KeystoreZeroizeTrigger::~KeystoreZeroizeTrigger()
{
	ScheduleClear();
}

int
KeystoreZeroizeTrigger::ConfigurePin()
{
	int ret = imx9_iomux_configure(GPIO_ZEROIZE_TRIGGER_MUX);

	if (ret < 0) {
		PX4_ERR("imx9_iomux_configure(%s) failed: %d", GPIO_ZEROIZE_TRIGGER_NAME, ret);
		return ret;
	}

	ret = px4_arch_configgpio(GPIO_ZEROIZE_TRIGGER);

	if (ret < 0) {
		PX4_ERR("px4_arch_configgpio(%s) failed: %d", GPIO_ZEROIZE_TRIGGER_NAME, ret);
		return ret;
	}

	return PX4_OK;
}

int
KeystoreZeroizeTrigger::ReadOnce()
{
	int ret = ConfigurePin();

	if (ret != PX4_OK) {
		return ret;
	}

	px4_usleep(1000);

	const bool level = px4_arch_gpioread(GPIO_ZEROIZE_TRIGGER);
	PX4_INFO("[GPIO %s at %s]", level ? "HIGH" : "LOW ", GPIO_ZEROIZE_TRIGGER_NAME);

	return PX4_OK;
}

int
KeystoreZeroizeTrigger::InspectKeystore(bool verbose)
{
	struct mtd_dev_s *nor = imx9_nor_device_get();

	if (nor == nullptr) {
		PX4_ERR("no NOR device");
		return -ENODEV;
	}

	char header[sizeof(kKeystoreSignature)] {};

	if (MTD_READ(nor, KEYSTORE_START_ADDRESS, sizeof(header), (uint8_t *)header) != (ssize_t)sizeof(header)) {
		PX4_ERR("keystore header read failed");
		return -EIO;
	}

	const bool header_ok = (memcmp(header, kKeystoreSignature, sizeof(kKeystoreSignature)) == 0);

	if (verbose) {
		PX4_INFO("keystore @0x%08x header: %s", (unsigned)KEYSTORE_START_ADDRESS,
			 header_ok ? "VALID (nxp93_keystore)" : "MISSING / ERASED");
	}

	int used = 0;

	for (unsigned idx = 0; idx < MAX_KEYS; idx++) {
		uint32_t meta[2] {};
		const off_t addr = (off_t)(KEYSTORE_START_SECTOR + idx + 1) * KEY_PAGE_SIZE;

		if (MTD_READ(nor, addr, sizeof(meta), (uint8_t *)meta) != (ssize_t)sizeof(meta)) {
			PX4_ERR("slot %u read failed", idx);
			return -EIO;
		}

		if (meta[0] == 0xFFFFFFFFu) {
			continue;
		}

		used++;

		if (verbose) {
			PX4_INFO("  slot %2u: len %4lu mask 0x%08lx", idx, (unsigned long)meta[0], (unsigned long)meta[1]);
		}
	}

	if (verbose) {
		PX4_INFO("populated slots: %d", used);
	}

	return used;
}

bool
KeystoreZeroizeTrigger::UpdateDebouncedLevel(bool raw)
{
	if (raw != _raw_prev) {
		_raw_prev = raw;
		_stable_count = 0;
		return false;
	}

	if (_stable_count < kDebounceSamples) {
		_stable_count++;
	}

	if (_stable_count < kDebounceSamples) {
		return false;
	}

	if (_have_level && raw == _level) {
		return false;
	}

	_level = raw;
	_have_level = true;

	return true;
}

bool
KeystoreZeroizeTrigger::EraseAndVerify()
{
	/* keystore_init() is ref-counted and already done by crypto_init(); calling it
	 * again only guarantees the flash device pointer is set before the erase. */
	keystore_init();
	keystore_erase();

	/* Verify: the header and every slot must read back as erased. */
	const int remaining = InspectKeystore(false);

	char header[sizeof(kKeystoreSignature)] {};
	struct mtd_dev_s *nor = imx9_nor_device_get();
	const bool header_gone = nor
				 && MTD_READ(nor, KEYSTORE_START_ADDRESS, sizeof(header), (uint8_t *)header) == (ssize_t)sizeof(header)
				 && memcmp(header, kKeystoreSignature, sizeof(kKeystoreSignature)) != 0;

	if (!header_gone || remaining != 0) {
		PX4_ERR("ZEROIZE: erase not verified (header %s, %d slots left)",
			header_gone ? "erased" : "still present", remaining);
		return false;
	}

	return true;
}

void
KeystoreZeroizeTrigger::Zeroize(const char *reason)
{
	PX4_WARN("##########################################################");
	PX4_WARN("ZEROIZE: triggered by %s", reason);
	PX4_WARN("ZEROIZE: erasing keystore (NOR 0x%08x, %d sectors)",
		 (unsigned)KEYSTORE_START_ADDRESS, MAX_KEYS + 1);

	bool erased = false;

	for (int attempt = 1; attempt <= kEraseAttempts && !erased; attempt++) {
		erased = EraseAndVerify();
	}

	if (!erased) {
		/* The keystore is still there, so a reset would only reboot into PX4. */
		_last_result = "FAILED (keystore still present)";
		PX4_ERR("ZEROIZE: keystore still present after %d attempts, NOT resetting", kEraseAttempts);
		return;
	}

	_last_result = "ok, resetting";
	PX4_WARN("ZEROIZE: keystore erased and verified. Resetting NOW.");
	PX4_WARN("##########################################################");

	/* Reset directly, with no conditions. Not px4_reboot_request(): its shutdown
	 * worker postpones the reset while the MOI agent reports a critical activity,
	 * so the zeroize would depend on something other than its trigger.
	 *
	 * REBOOT_TO_BOOTLOADER, not REBOOT_REQUEST: on imx9 REBOOT_REQUEST is a warm
	 * reboot (board_warm_reboot(): PX4 restarts from RAM, the bootloader never
	 * runs). REBOOT_TO_BOOTLOADER is a real SoC reset through boot ROM ->
	 * bootloader, which finds the erased keystore, fails the PX4 signature check
	 * and stays in the bootloader. */
	px4_usleep(kResetDelayUs);
	board_reset(REBOOT_TO_BOOTLOADER);
}

void
KeystoreZeroizeTrigger::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	/* Button trigger. Its only condition is its own: debounced HIGH for 5 s.
	 * Zeroize() itself has no conditions (not the armed state, not anything else). */
	const bool raw = px4_arch_gpioread(GPIO_ZEROIZE_TRIGGER);
	const bool had_level = _have_level;

	if (UpdateDebouncedLevel(raw)) {
		PX4_INFO("[GPIO %s at %s]", _level ? "HIGH" : "LOW ", GPIO_ZEROIZE_TRIGGER_NAME);

		if (had_level) {
			_edges++;
		}
	}

	if (!_level) {
		if (_hold_count > 0 && !_latched) {
			PX4_INFO("zeroize hold released after %.1f s, cancelled",
				 (double)(_hold_count * kPollIntervalUs) / 1e6);
		}

		_hold_count = 0;
		_latched = false;
		_last_progress_s = 0;
		return;
	}

	if (_latched) {
		return;
	}

	_hold_count++;

	/* Countdown once per second, so the operator can see it is about to fire. */
	const uint8_t held_s = (uint8_t)((_hold_count * kPollIntervalUs) / 1000000);

	if (held_s != _last_progress_s) {
		_last_progress_s = held_s;
		PX4_WARN("zeroize trigger held %u s / %u s",
			 held_s, (unsigned)((kHoldSamples * kPollIntervalUs) / 1000000));
	}

	if (_hold_count >= kHoldSamples) {
		_latched = true;
		_hold_count = 0;
		Zeroize("button held 5 s on " GPIO_ZEROIZE_TRIGGER_NAME);
	}
}

int
KeystoreZeroizeTrigger::Start()
{
	int ret = ConfigurePin();

	if (ret != PX4_OK) {
		return ret;
	}

	ScheduleOnInterval(kPollIntervalUs);

	return PX4_OK;
}

int
KeystoreZeroizeTrigger::task_spawn(int argc, char *argv[])
{
	KeystoreZeroizeTrigger *instance = new KeystoreZeroizeTrigger();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -ENOMEM;
	}

	int ret = instance->Start();

	if (ret != PX4_OK) {
		delete instance;
		return ret;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	PX4_WARN("ACTIVE: holding %s high for %.1f s WILL ERASE THE KEYSTORE AND RESET (armed or not)",
		 GPIO_ZEROIZE_TRIGGER_NAME, (double)(kHoldSamples * kPollIntervalUs) / 1e6);

	return ret;
}

int
KeystoreZeroizeTrigger::print_status()
{
	PX4_INFO("polling %s at %lu Hz, hold %.1f s to zeroize", GPIO_ZEROIZE_TRIGGER_NAME,
		 (unsigned long)(1000000 / kPollIntervalUs),
		 (double)(kHoldSamples * kPollIntervalUs) / 1e6);

	if (_have_level) {
		PX4_INFO("level: %s, edges seen: %lu", _level ? "HIGH" : "LOW", (unsigned long)_edges);

	} else {
		PX4_INFO("level: not sampled yet");
	}

	if (_latched) {
		PX4_INFO("hold: latched (fired)");

	} else if (_hold_count > 0) {
		PX4_INFO("hold: %.1f / %.1f s", (double)(_hold_count * kPollIntervalUs) / 1e6,
			 (double)(kHoldSamples * kPollIntervalUs) / 1e6);

	} else {
		PX4_INFO("hold: idle");
	}

	PX4_INFO("last result: %s", _last_result);

	InspectKeystore(true);

	return PX4_OK;
}

int
KeystoreZeroizeTrigger::custom_command(int argc, char *argv[])
{
	if (argc > 0 && !strcmp(argv[0], "read")) {
		return ReadOnce();
	}

	if (argc > 0 && !strcmp(argv[0], "keystore")) {
		return InspectKeystore(true) < 0 ? PX4_ERROR : PX4_OK;
	}

	return print_usage("unknown command");
}

int
KeystoreZeroizeTrigger::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PROOF OF CONCEPT. Erases the NOR keystore and reboots when a GPIO input is held.

The pin is TELEM1 CTS: pad GPIO_IO16 used as GPIO2_16, base board connector
J13 pin 4 (3.3 V logic, internal pull-down; J13 pin 1 is 5 V, do not use it).

Polled at 50 Hz, debounced over 60 ms. Holding it high for 5 s erases the
keystore (header + all 50 slots), verifies the erase and resets the SoC, with
no other condition: armed or disarmed, in flight or on the ground. The
bootloader then cannot verify PX4 and stays in the bootloader. Recovery
requires re-flashing a keystore backup of the same board.

### Examples
Sample the pin once (never erases):
$ keystore_zeroize_trigger read

Show the keystore slots currently in NOR (read only):
$ keystore_zeroize_trigger keystore

Arm the trigger:
$ keystore_zeroize_trigger start
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("keystore_zeroize_trigger", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("read", "configure the pin, sample it once, print the level");
	PRINT_MODULE_USAGE_COMMAND_DESCR("keystore", "print keystore header and populated slots (read only)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int keystore_zeroize_trigger_main(int argc, char *argv[]);

int
keystore_zeroize_trigger_main(int argc, char *argv[])
{
	return KeystoreZeroizeTrigger::main(argc, argv);
}
