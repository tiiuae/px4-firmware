#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include <px4_arch/imxrt_flexspi_nor_flash.h>
#include <px4_arch/imxrt_romapi.h>
#include <common_src/state_ctrl.h>

#include "board_config.h"

#define NOR_PAGE   256
#define NOR_SECTOR 4096

struct slot_dev_s {
	uint32_t offset;
	uint8_t slot;
	bool writer;
	uint32_t erased_start;
	uint32_t erased_end;
	mutex_t lock;
};

extern struct flexspi_nor_config_s g_bootConfig;

static struct slot_dev_s g_slots[2] = {
	{.offset = BOARD_SLOT_A_OFFSET, .slot = 0, .lock = NXMUTEX_INITIALIZER},
	{.offset = BOARD_SLOT_A_OFFSET + BOARD_SLOT_B_OFFSET, .slot = 1, .lock = NXMUTEX_INITIALIZER},
};

locate_code(".ramfunc")
static void nor_op(uint32_t offset, const uint32_t *page, uint32_t *out, size_t len)
{
	irqstate_t flags = enter_critical_section();

	if (out != NULL) {
		ROM_FLEXSPI_NorFlash_Read(1, &g_bootConfig, out, offset, len);

	} else if (page != NULL) {
		ROM_FLEXSPI_NorFlash_ProgramPage(1, &g_bootConfig, offset, page);

	} else {
		ROM_FLEXSPI_NorFlash_Erase(1, &g_bootConfig, offset, len);
	}

	ROM_FLEXSPI_NorFlash_ClearCache(1);
	leave_critical_section(flags);
}

static bool slot_active(const struct slot_dev_s *dev)
{
	partition_t partition;

	return statectrl_get_partition(&partition) < 0 || partition.active == dev->slot;
}

static int slot_open(struct file *filep)
{
	struct slot_dev_s *dev = filep->f_inode->i_private;
	int ret = OK;

	if ((filep->f_oflags & O_WROK) == 0) {
		return OK;
	}

	nxmutex_lock(&dev->lock);

	if (slot_active(dev)) {
		ret = -EACCES;

	} else if (dev->writer) {
		ret = -EBUSY;

	} else {
		dev->writer = true;
		dev->erased_start = dev->erased_end = 0;
	}

	nxmutex_unlock(&dev->lock);
	return ret;
}

static int slot_close(struct file *filep)
{
	struct slot_dev_s *dev = filep->f_inode->i_private;

	if (filep->f_oflags & O_WROK) {
		nxmutex_lock(&dev->lock);
		dev->writer = false;
		nxmutex_unlock(&dev->lock);
	}

	return OK;
}

static ssize_t slot_read(struct file *filep, char *buffer, size_t buflen)
{
	struct slot_dev_s *dev = filep->f_inode->i_private;
	uint32_t page[NOR_PAGE / sizeof(uint32_t)];
	size_t done = 0;

	while (done < buflen && filep->f_pos < BOARD_SLOT_SIZE) {
		uint32_t pos = filep->f_pos;
		uint32_t base = pos & ~(NOR_PAGE - 1);
		size_t n = NOR_PAGE - (pos - base);

		n = n < buflen - done ? n : buflen - done;
		nor_op(dev->offset + base, NULL, page, NOR_PAGE);
		memcpy(buffer + done, (uint8_t *)page + (pos - base), n);
		filep->f_pos += n;
		done += n;
	}

	return done;
}

static ssize_t slot_write(struct file *filep, const char *buffer, size_t buflen)
{
	struct slot_dev_s *dev = filep->f_inode->i_private;
	uint32_t page[NOR_PAGE / sizeof(uint32_t)];
	uint32_t verify[NOR_PAGE / sizeof(uint32_t)];
	size_t done = 0;

	if (slot_active(dev)) {
		return -EACCES;
	}

	while (done < buflen) {
		uint32_t pos = filep->f_pos;
		uint32_t sector = pos & ~(NOR_SECTOR - 1);
		uint32_t base = pos & ~(NOR_PAGE - 1);
		size_t off = pos - base;
		size_t n = NOR_PAGE - off;

		if (pos >= BOARD_SLOT_SIZE) {
			return done > 0 ? (ssize_t)done : -EFBIG;
		}

		if (sector < dev->erased_start || sector >= dev->erased_end) {
			nor_op(dev->offset + sector, NULL, NULL, NOR_SECTOR);

			if (sector == dev->erased_end && dev->erased_end > dev->erased_start) {
				dev->erased_end += NOR_SECTOR;

			} else {
				dev->erased_start = sector;
				dev->erased_end = sector + NOR_SECTOR;
			}
		}

		n = n < buflen - done ? n : buflen - done;
		memset(page, 0xff, sizeof(page));
		memcpy((uint8_t *)page + off, buffer + done, n);
		nor_op(dev->offset + base, page, NULL, NOR_PAGE);
		nor_op(dev->offset + base, NULL, verify, NOR_PAGE);

		if (memcmp((uint8_t *)page + off, (uint8_t *)verify + off, n) != 0) {
			return -EIO;
		}

		filep->f_pos += n;
		done += n;
	}

	return done;
}

static off_t slot_seek(struct file *filep, off_t offset, int whence)
{
	off_t pos;

	switch (whence) {
	case SEEK_SET:
		pos = offset;
		break;

	case SEEK_CUR:
		pos = filep->f_pos + offset;
		break;

	case SEEK_END:
		pos = BOARD_SLOT_SIZE + offset;
		break;

	default:
		return -EINVAL;
	}

	if (pos < 0 || pos > BOARD_SLOT_SIZE) {
		return -EINVAL;
	}

	filep->f_pos = pos;
	return pos;
}

static const struct file_operations g_slot_ops = {
	.open = slot_open,
	.close = slot_close,
	.read = slot_read,
	.write = slot_write,
	.seek = slot_seek,
};

int imxrt_nor_slots_initialize(void)
{
	int ret = register_driver("/dev/mtd_px4_1", &g_slot_ops, 0600, &g_slots[0]);

	if (ret >= 0) {
		ret = register_driver("/dev/mtd_px4_2", &g_slot_ops, 0600, &g_slots[1]);
	}

	return ret;
}
