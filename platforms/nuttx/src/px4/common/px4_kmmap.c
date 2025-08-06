/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mman.h>
#include <sys/types.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mm/kmap.h>
#include <nuttx/pgalloc.h>

/* This is naughty, but only option as these are in NuttX private headers */

extern int inode_lock(void);
extern void inode_unlock(void);

struct shmfs_object_s {
	size_t length;
	void *paddr;
};

void *px4_mmap(void *start, size_t length, int prot, int flags, int fd, off_t offset)
{
	struct file *filep;
	struct shmfs_object_s *object;
	void **pages;
	void *vaddr;
	unsigned int npages;
	int ret;

	if (file_get(fd, &filep) < 0) {
		ret = -EBADF;
		goto errout;
	}

	ret = inode_lock();

	if (ret < 0) {
		goto errout;
	}

	/* Return the physical address */

	object = (struct shmfs_object_s *)filep->f_inode->i_private;

	if (!object) {
		ret = -EINVAL;
		goto errout_with_lock;
	}

	/* Map the object to kernel */

	pages = &object->paddr;
	npages = MM_NPAGES(object->length);

	/* Do the mapping */

	vaddr = kmm_map(pages, npages, PROT_READ | PROT_WRITE);

	if (!vaddr) {
		ret = -ENOMEM;
		goto errout_with_lock;
	}

	filep->f_inode->i_crefs++;
	inode_unlock();
	file_put(filep);
	return vaddr;

errout_with_lock:
	inode_unlock();
errout:
	file_put(filep);
	set_errno(-ret);
	return MAP_FAILED;
}

int px4_munmap(void *start, size_t length)
{
	kmm_unmap(start);
	return OK;
}
