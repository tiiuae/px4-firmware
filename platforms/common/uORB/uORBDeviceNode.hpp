/****************************************************************************
 *
 *   Copyight (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include <sys/shm.h>
#include <sys/mman.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <limits.h>

#include "uORB.h"
#include "uORBCommon.hpp"
#include <uORB/topics/uORBTopics.hpp>
#include <px4_platform_common/sem.h>

#include <containers/List.hpp>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/px4_config.h>

#include "IndexedStack.hpp"

#if defined(CONFIG_BUILD_FLAT)
#define MAX_EVENT_WAITERS 0 // dynamic
typedef void *uorb_cb_handle_t;
#define UORB_INVALID_CB_HANDLE nullptr
#define uorb_cb_handle_valid(x) ((x) != nullptr)
#else
#if defined(CONFIG_BUILD_KERNEL) || defined(__PX4_POSIX)
#define MAX_EVENT_WAITERS 32
#else
#define MAX_EVENT_WAITERS 6
#endif
#define UORB_INVALID_CB_HANDLE -1
typedef int8_t uorb_cb_handle_t;
#define uorb_cb_handle_valid(x) ((x) >= 0)
#define CB_ALIVE_MAX_VALUE 50
#endif

#define CB_LIST_T struct EventWaitItem, uorb_cb_handle_t, MAX_EVENT_WAITERS

namespace uORBTest
{
class UnitTest;
}

namespace uORB
{

class SubscriptionCallback;

/**
 * Per-object device instance.
 */
class DeviceNode
{
public:
	// Open a node, either existing or create a new one
	static orb_advert_t nodeOpen(const ORB_ID id, const uint8_t instance, bool create);
	static int nodeClose(orb_advert_t &handle);

	~DeviceNode();

	// no copy, assignment, move, move assignment
	DeviceNode(const DeviceNode &) = delete;
	DeviceNode &operator=(const DeviceNode &) = delete;
	DeviceNode(DeviceNode &&) = delete;
	DeviceNode &operator=(DeviceNode &&) = delete;

	bool operator<=(const DeviceNode &rhs) const { return (strcmp(get_devname(), rhs.get_devname()) <= 0); }

	/**
	 * Method to publish a data to this node.
	 */
	static ssize_t    publish(const orb_metadata *meta, orb_advert_t &handle, const void *data);

	static orb_advert_t orb_advertise(const ORB_ID id, int instance, bool publisher);

	static int        orb_unadvertise(orb_advert_t &handle, bool publisher);

#ifdef CONFIG_ORB_COMMUNICATOR
	static const char *get_name(orb_advert_t handle)
	{
		if (orb_advert_valid(handle)) {
			return node(handle)->get_name();

		} else {
			return nullptr;
		}
	}

	/**
	 * processes a request for topic advertisement from remote
	 * @param meta
	 *   The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @return
	 *   0 = success
	 *   otherwise failure.
	 */
	static int16_t topic_advertised(const orb_metadata *meta);

	/**
	 * processes a request for add subscription from remote
	 * @return
	 *   0 = success
	 *   otherwise failure.
	 */
	int16_t process_add_subscription(orb_advert_t &handle);

	/**
	 * processes a request to remove a subscription from remote.
	 */
	int16_t process_remove_subscription(orb_advert_t &handle);

	/**
	 * processed the received data message from remote.
	 */
	int16_t process_received_message(orb_advert_t &handle, int32_t length, uint8_t *data);
#endif /* CONFIG_ORB_COMMUNICATOR */

	/**
	  * Add the subscriber to the node's list of subscribers.  If there is
	  * remote proxy to which this subscription needs to be sent, it will
	  * done via uORBCommunicator::IChannel interface.
	  * @param initial_generation
	  *   the handle to the orb to which the subscriber is added
	  *   the subscriber's initial generation
	  * @return
	  *   subscriber's handle to the orb
	  */
	static orb_advert_t add_subscriber(ORB_ID orb_id, uint8_t instance, unsigned *initial_generation,
					   bool advertise);

	/**
	 * Removes the subscriber from the list.  Also notifies the remote
	 * if there a uORBCommunicator::IChannel instance.
	 * @param handle
	 *   the handle to the orb from which the subscriber is removed
	 *   the handle is invalidated after a succesful removal
	 */
	int8_t remove_subscriber(orb_advert_t &handle, bool advertiser);

	/**
	 * Print statistics
	 * @param max_topic_length max topic name length for printing
	 * @return true if printed something, false otherwise
	 */
	bool print_statistics(int max_topic_length);

	uint8_t get_queue_size() const { return get_orb_meta(_orb_id)->o_queue; }
	/**
	 * Get count of subscribers for this topic
	 * @return number of active subscribers
	 */
	int8_t subscriber_count() const { return _subscriber_count; }

	/**
	 * Get count of publishers for this topic
	 * @return number of publishers which have advertised this topic
	 */
	int8_t publisher_count() const { return _publisher_count; }

	/**
	 * Returns the number of updated data relative to the parameter 'generation'
	 * We can get the correct value regardless of wrap-around or not.
	 * @param generation The generation of subscriber
	 */
	unsigned updates_available(unsigned generation) const { return _data_valid ? _generation - generation : 0; }

	const orb_metadata *get_meta() const { return get_orb_meta(_orb_id); }

	size_t get_size() { return get_orb_size(_orb_id); }
	static size_t get_orb_size(ORB_ID id) { return sizeof(DeviceNode) + get_orb_meta(id)->o_size * get_orb_meta(id)->o_queue + data_alignment_padding; }

	ORB_ID id() const { return _orb_id; }

	const char *get_name() const { return get_orb_meta(_orb_id)->o_name; }

	uint8_t get_instance() const { return _instance; }

	/**
	 * Copies data and the corresponding generation
	 * from a node to the buffer provided.
	 *
	 * @param dst
	 *   The buffer into which the data is copied.
	 * @param generation
	 *   The generation that was copied.
	 * @return bool
	 *   Returns true if the data was copied.
	 */
	bool copy(void *dst, orb_advert_t &handle, unsigned &generation);

	static bool register_callback(orb_advert_t &node_handle, SubscriptionCallback *callback_sub, int8_t poll_lock,
				      hrt_abstime last_update, uint32_t interval_us, uorb_cb_handle_t &cb_handle)
	{
		return node(node_handle)->_register_callback(callback_sub, poll_lock, last_update, interval_us, cb_handle);
	}

	static int unregister_callback(orb_advert_t &node_handle, uorb_cb_handle_t &cb_handle)
	{
		return node(node_handle)->_unregister_callback(cb_handle);
	}

	void *operator new (size_t, void *p)
	{
		return p;
	}

	void operator delete (void *p)
	{
	}

	const char *get_devname() const {return _devname;}

#ifndef CONFIG_BUILD_FLAT
	static bool cb_dequeue(orb_advert_t &node_handle, uorb_cb_handle_t cb)
	{
		IndexedStackHandle<CB_LIST_T> callbacks(node(node_handle)->_callbacks);
		EventWaitItem *item = callbacks.peek(cb);

		if (__atomic_load_n(&item->cb_triggered, __ATOMIC_SEQ_CST) > 0) {
			__atomic_fetch_sub(&item->cb_triggered, 1, __ATOMIC_SEQ_CST);
			return true;
		}

		return false;
	}
#endif

private:
	friend uORBTest::UnitTest;

	const ORB_ID _orb_id;

	bool _data_valid{false}; /**< At least one valid data */
	unsigned _generation{0};  /**< object generation count */

	struct EventWaitItem {
		hrt_abstime last_update;
#ifdef CONFIG_BUILD_FLAT
		class SubscriptionCallback *subscriber;
#else
		uint32_t cb_triggered;
#endif
		uint32_t interval_us;
		uorb_cb_handle_t next; // List ptr
		int8_t lock;
	};

	IndexedStack<CB_LIST_T> _callbacks;

	inline ssize_t write(const char *buffer, const orb_metadata *meta, orb_advert_t &handle);
	static int callback_thread(int argc, char *argv[]);
	struct SubscriptionCallback *callback_sub;

	class MappingCache
	{
	public:
		struct MappingCacheListItem {
			MappingCacheListItem *next;
			orb_advert_t handle;
		};

		// This list is process specific in kernel build and global in in flat
		static MappingCacheListItem *g_cache;
		static px4_sem_t g_cache_lock;

		static void init(void)
		{
			static bool initialized = false;

			if (!initialized) {
				px4_mutex_init(&g_cache_lock, 0);
				initialized = true;
			}
		}
		static orb_advert_t get(ORB_ID orb_id, uint8_t instance);
		static orb_advert_t map_node(ORB_ID orb_id, uint8_t instance, int shm_fd);
		static orb_advert_t map_data(orb_advert_t handle, int shm_fd, bool publisher);
		static bool del(const orb_advert_t &handle);

		static void lock()
		{
			init();
			do {} while (px4_sem_wait(&g_cache_lock) != 0);
		}
		static void unlock() { px4_sem_post(&g_cache_lock); }
	};

	const uint8_t _instance; /**< orb multi instance identifier */
	uint8_t _queue_size{0}; /**< maximum number of elements in the queue */
	int8_t _subscriber_count{0}; /**< how many subscriptions there are */
	int8_t _publisher_count{0}; /**< how many publishers have advertised this topic */
	int8_t _advertiser_count{0}; /**< how many total advertisers this topic has */

	DeviceNode(const ORB_ID id, const uint8_t instance, const char *path);

	int advertise(bool publisher);
	int unadvertise(bool publisher);

	/**
	 * Change the size of the queue.
	 * @return PX4_OK if queue size successfully set
	 */
	int update_queue_size(unsigned int queue_size);

	void _add_subscriber(unsigned *initial_generation);

	inline static DeviceNode *node(const orb_advert_t &handle) { return static_cast<DeviceNode *>(handle); }
	inline static uint8_t *node_data(const orb_advert_t &handle) { return static_cast<DeviceNode *>(handle)->_data + data_alignment_padding; }

	bool _register_callback(SubscriptionCallback *callback_sub, int8_t poll_lock, hrt_abstime last_update,
				uint32_t interval_us, uorb_cb_handle_t &cb_handle);
	int _unregister_callback(uorb_cb_handle_t &cb_handle);

	/**
	 * Mutex for protecting node's internal data
	 */

	void		lock() { do {} while (px4_sem_wait(&_lock) != 0); }
	void		unlock() { px4_sem_post(&_lock); }
	void		lock_cb() { do {} while (px4_sem_wait(&_cb_lock) != 0); }
	void		unlock_cb() { px4_sem_post(&_cb_lock); }

	px4_sem_t	_lock;     /**< lock to protect access to uORB data */
	px4_sem_t	_cb_lock;  /**< lock to protect access to callback queue */

#ifdef CONFIG_BUILD_FLAT
	char *_devname;
#else
	char _devname[NAME_MAX + 1];
#endif
	static const unsigned data_alignment_padding;
	uint8_t _data[];
};
} //namespace uORB
