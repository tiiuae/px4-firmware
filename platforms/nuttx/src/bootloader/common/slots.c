#include <nuttx/config.h>

#include <string.h>

#include <common_src/devstate.h>

#include "slots.h"

static devstate_t g_ds;

static void slots_store(void)
{
	devstate_fill_crc16_ccitt(&g_ds);
	board_devstate_write((const uint8_t *)&g_ds, g_ds.header.struct_len);
}

static void slots_load(void)
{
	int n = board_devstate_read((uint8_t *)&g_ds, sizeof(g_ds));

	if (n < DEVSTATE_HEADER_LEN || g_ds.header.struct_len > sizeof(g_ds) ||
	    !devstate_check_crc16_ccitt(&g_ds)) {
		memset(&g_ds, 0, sizeof(g_ds));

	} else if (g_ds.header.version.major != DEVSTATE_MAJOR_VERSION) {
		memset((uint8_t *)&g_ds + sizeof(g_ds.header), 0, sizeof(g_ds) - sizeof(g_ds.header));

	} else if (g_ds.header.version.minor >= DEVSTATE_MINOR_VERSION) {
		return;
	}

	g_ds.header.version.major = DEVSTATE_MAJOR_VERSION;
	g_ds.header.version.minor = DEVSTATE_MINOR_VERSION;
	g_ds.header.struct_len = DEVSTATE_STRUCT_LEN;
	slots_store();
}

static bool slot_verify(int slot)
{
	board_slot_select(slot);
	return board_slot_verify();
}

int slots_select(void)
{
	slots_load();

	fw_update_t *fu = &g_ds.header.fw_update;
	int active = g_ds.partition.active == PARTITION_2 ? PARTITION_2 : PARTITION_1;
	int other = active == PARTITION_1 ? PARTITION_2 : PARTITION_1;
	int slot = active;

	if (fu->validation && board_slot_bootable(other)) {
		if (fu->new_image_trial) {
			board_slot_erase(other);
			fu->new_image_trial = 0;

		} else {
			fu->new_image_trial = 1;
			slot = other;
		}

		slots_store();

	} else if (fu->new_image_trial) {
		fu->new_image_trial = 0;
		slots_store();
	}

	if (slot == active && !board_slot_bootable(active) && board_slot_bootable(other)) {
		g_ds.partition.active = other;
		slots_store();
		slot = other;
	}

	int alt = slot == PARTITION_1 ? PARTITION_2 : PARTITION_1;

	if (slot_verify(slot)) {
		return slot;
	}

	if (board_slot_bootable(alt) && slot_verify(alt)) {
		return alt;
	}

	board_slot_select(slot);
	return -1;
}

void slots_reset(void)
{
	slots_load();
	g_ds.partition.active = PARTITION_1;
	g_ds.header.fw_update.validation = 0;
	g_ds.header.fw_update.new_image_trial = 0;
	slots_store();
	board_slot_select(PARTITION_1);
}
