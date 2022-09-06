/****************************************************************************
 *
 *   Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 *   NuttX SocketCAN port Copyright (C) 2022 NXP Semiconductors
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

#pragma once

#include <uavcan_nuttx/thread.hpp>
#include <uavcan_nuttx/clock.hpp>
#include <uavcan/driver/can.hpp>

#include <sys/time.h>
#include <sys/socket.h>

#include <nuttx/can.h>
#include <netpacket/can.h>

namespace uavcan_socketcan
{

class CanIface : public uavcan::ICanIface
	, uavcan::Noncopyable
{
	int               _fd{-1};
	bool              _can_fd{false};

	//// Send msg structure
	struct iovec       _send_iov {};
	struct canfd_frame _send_frame {};
	struct msghdr      _send_msg {};
	struct cmsghdr     *_send_cmsg {};
	struct timeval     *_send_tv {};  /* TX deadline timestamp */
	uint8_t            _send_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	//// Receive msg structure
	struct iovec       _recv_iov {};
	struct canfd_frame _recv_frame {};
	struct msghdr      _recv_msg {};
	struct cmsghdr     *_recv_cmsg {};
	uint8_t            _recv_control[sizeof(struct cmsghdr) + sizeof(struct timeval)] {};

	SystemClock clock;

public:
	uavcan::uint32_t socketInit(const char *can_iface_name);

	int close();

	uavcan::int16_t send(const uavcan::CanFrame &frame,
			     uavcan::MonotonicTime tx_deadline,
			     uavcan::CanIOFlags flags) override;

	uavcan::int16_t receive(uavcan::CanFrame &out_frame,
				uavcan::MonotonicTime &out_ts_monotonic,
				uavcan::UtcTime &out_ts_utc,
				uavcan::CanIOFlags &out_flags) override;

	uavcan::int16_t configureFilters(const uavcan::CanFilterConfig *filter_configs,
					 uavcan::uint16_t num_configs) override;

	uavcan::uint64_t getErrorCount() const override;

	uavcan::uint16_t getNumFilters() const override;

	int getFD();
};

/**
 * This class implements CAN driver interface for libuavcan.
 * No configuration needed other than CAN baudrate.
 */
class CanDriver
	: public uavcan::ICanDriver
	, uavcan::Noncopyable
{
	BusEvent update_event_;
	CanIface if_[UAVCAN_SOCKETCAN_NUM_IFACES];
	SystemClock clock;
	struct pollfd pfds[UAVCAN_SOCKETCAN_NUM_IFACES];

public:
	CanDriver() : update_event_(*this)
	{}

	uavcan::int32_t initIface(uint32_t index, const char *name)
	{
		if (index > (UAVCAN_SOCKETCAN_NUM_IFACES - 1)) {
			return -1;
		}
		return if_[index].socketInit(name);
	}

	int closeAllIface()
	{
		int ret = 0;
		for (int i = 0; i < UAVCAN_SOCKETCAN_NUM_IFACES; i++) {
			if((ret = if_[i].close())) {
				return ret;
			}
		}
		return ret;
	}

	/**
	 * Returns negative value if the requested baudrate can't be used.
	 * Returns zero if OK.
	 */
	int init();

	/**
	 * Returns the number of times the RX queue was overrun.
	 */
	uavcan::uint32_t getRxQueueOverflowCount() const;

	/**
	 * Whether the controller is currently in bus off state.
	 * Note that the driver recovers the CAN controller from the bus off state automatically!
	 * Therefore, this method serves only monitoring purposes and is not necessary to use.
	 */
	bool isInBusOffState() const;

	uavcan::int16_t select(uavcan::CanSelectMasks &inout_masks,
			       const uavcan::CanFrame * (&)[uavcan::MaxCanIfaces],
			       uavcan::MonotonicTime blocking_deadline) override;

	uavcan::ICanIface *getIface(uavcan::uint8_t iface_index) override;

	uavcan::uint8_t getNumIfaces() const override;

	BusEvent &updateEvent() { return update_event_; }
};


template <unsigned RxQueueCapacity = 128>
class CanInitHelper
{
	//CanRxItem queue_storage_[UAVCAN_KINETIS_NUM_IFACES][RxQueueCapacity];

public:
	enum { BitRateAutoDetect = 0 };

	CanDriver driver;

	CanInitHelper(uint32_t unused = 0x7) :
		driver()
	{
	}

	/**
	 * @return  Negative value on error; non-negative on success. Refer to constants Err*.
	 */
	int init()
	{
		return driver.init();
	}
};

}
