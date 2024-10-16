/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

/**
 * @file multi_instance_module.h
 */

#pragma once

#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <containers/List.hpp>
#include <systemlib/px4_macros.h>

#ifdef __cplusplus

#include <cstring>

const int MAX_INSTANCE_COUNT = 6;

#ifdef __PX4_NUTTX
#  define MAX_NAME_SIZE CONFIG_TASK_NAME_SIZE
#else
#  define MAX_NAME_SIZE 32
#endif

/**
 * @class MultiInstanceModuleBase
 *      Base class for multi-instance modules, implementing common functionality,
 *      such as 'start', 'stop', 'stop-all' and 'status' commands.
 *
 *      The class is implemented as curiously recurring template pattern (CRTP).
 *      It allows to have a static objects in the base class that is different for
 *      each module type, and call static methods from the base class.
 *
 * @note Required methods for a derived class:
 * When running in its own thread:
 *      static int task_spawn(int argc, char *argv[])
 *      {
 *              // call px4_task_spawn_cmd() with &run_trampoline as startup method
 *              // optional: wait until _object is not null, which means the task got initialized (use a timeout)
 *              // set _task_id and return 0
 *              // on error return != 0 (and _task_id must be -1)
 *      }
 *
 *      static T *instantiate(int argc, char *argv[])
 *      {
 *              // this is called from within the new thread, from run_trampoline()
 *              // parse the arguments
 *              // create a new object T & return it
 *              // or return nullptr on error
 *      }
 *
 *      static int custom_command(int argc, char *argv[])
 *      {
 *              // support for custom commands
 *              // it none are supported, just do:
 *              return print_usage("unrecognized command");
 *      }
 *
 *      static int print_usage(const char *reason = nullptr)
 *      {
 *              // use the PRINT_MODULE_* methods...
 *      }
 *
 * When running on the work queue:
 * - custom_command & print_usage as above
 *      static int task_spawn(int argc, char *argv[]) {
 *              // parse the arguments
 *              // set _object (here or from the work_queue() callback)
 *              // call work_queue() (with a custom cycle trampoline)
 *              // optional: wait until _object is not null, which means the task got initialized (use a timeout)
 *              // set _task_id to task_id_is_work_queue and return 0
 *              // on error return != 0 (and _task_id must be -1)
 *      }
 */


template<class T>
class InstanceNode : public ListNode<InstanceNode<T> *>
{
public:
	InstanceNode() {};
	~InstanceNode() {}

	char _name[32]{0};
	px4::atomic<T *> _obj{nullptr};
	int _tid{-1};
};


template<class T>
class InstanceStorage
{
public:
	InstanceStorage() {
		_multi_inst_mod_mutex = PTHREAD_MUTEX_INITIALIZER;
	};
	~InstanceStorage() {
		for (auto t: _instances) {
			if (t->_obj.load()) {
				delete t->_obj.load();
				t->_obj.store(nullptr);
			}
		}
		_instances.clear();
	}

	pthread_mutex_t _multi_inst_mod_mutex;
	List<InstanceNode<T> *> _instances;
};

template<class T>
InstanceStorage<T> *_storage{nullptr};

template<class T>
class MultiInstanceModuleBase
{
public:

	MultiInstanceModuleBase() : _task_should_exit{false} {}
	~MultiInstanceModuleBase() {}

	/**
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 if successful, -1 otherwise.
	 */
	static int main(int argc, char *argv[])
	{
		if (argc <= 1 ||
		    strcmp(argv[1], "-h")    == 0 ||
		    strcmp(argv[1], "help")  == 0 ||
		    strcmp(argv[1], "info")  == 0 ||
		    strcmp(argv[1], "usage") == 0) {
			return T::print_usage();
		}

		if (strcmp(argv[1], "start") == 0) {

			// Pass the 'start' argument too, because later on px4_getopt() will ignore the first argument.
			if (argc > 2) {
				if (!_storage<T>) {
					_storage<T> = new InstanceStorage<T>();
				}
				lock_module();
				if (_storage<T>->_instances.size() == MAX_INSTANCE_COUNT) {
					PX4_ERR("Maximum multi-module instance count of %d reached.", MAX_INSTANCE_COUNT);
					unlock_module();
					return 1;
				}

				if (get_instance_by_name(argv[2])) {
					PX4_ERR("Instance name already in use");
					unlock_module();
					return 1;
				}

				// Create new instance placeholder into NodeList with instance name
				InstanceNode<T> *node = new InstanceNode<T>();
				snprintf(node->_name, sizeof(node->_name), "%s", argv[2]);
				_storage<T>->_instances.add(node);
				unlock_module();

				// Keep instance name argument
				return start_command_base(argc - 2, argv + 2);
			}
			return T::print_usage();
		}

		if (!_storage<T>) {
			PX4_INFO("not running");
			return 0;
		}

		if (strcmp(argv[1], "status") == 0) {
			int ret = 0;
			lock_module();
			if (argc > 2) {
				ret = status_command(get_instance_by_name(argv[2]));

			} else {
				if (_storage<T>->_instances.size() > 0) {
					PX4_INFO("Instance count: %lu", _storage<T>->_instances.size());
					for (auto node: _storage<T>->_instances) {
						if (status_command(node) != 0) {
							ret = 1;
						}
					}
				} else {
					PX4_INFO("not running");
				}
			}
			unlock_module();
			return ret;
		}

		if (strcmp(argv[1], "stop") == 0) {
			if (argc > 2) {
				lock_module();
				InstanceNode<T> *node = get_instance_by_name(argv[2]);
				if (node) {
					int ret = stop_command(node);
					unlock_module();
					clean_storage();
					return ret;
				}
				unlock_module();
			}
			return T::print_usage();
		}

		if (strcmp(argv[1], "stop-all") == 0) {
			int ret = 0;
			lock_module();
			while (!_storage<T>->_instances.empty()) {
				auto node = _storage<T>->_instances.getHead();
				if (stop_command(node) != 0) {
					ret = 1;
				}
			}
			unlock_module();
			clean_storage();
			return ret;
		}

		lock_module(); // Lock here, as the method could access _object.
		int ret = T::custom_command(argc - 1, argv + 1);
		unlock_module();

		return ret;
	}

	static void clean_storage()
	{
		if (_storage<T>) {
			if (_storage<T>->_instances.empty()) {
				delete _storage<T>;
				_storage<T> = nullptr;
			}
		}
	}

	/**
	 * @brief Entry point for px4_task_spawn_cmd() if the module runs in its own thread.
	 *        It does:
	 *        - instantiate the object
	 *        - store new instance to list
	 *        - call run() on it to execute the main loop
	 *        - cleanup: delete the object
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 if successful, -1 otherwise.
	 */
	static int run_trampoline(int argc, char *argv[])
	{
		int ret = 0;

		// We don't need the task name at this point.
		argc -= 1;
		argv += 1;

		char* name = argv[0];
		T *object = T::instantiate(argc, argv);

		if (object) {
			lock_module();
			// Get instance node by name and store instance object
			InstanceNode<T> *node = get_instance_by_name(name);
			if (!node) {
				PX4_ERR("run: instance not found!");
				delete object;
				unlock_module();
				return -1;
			}
			node->_tid = px4_getpid();
			node->_obj.store(object);

			unlock_module();

			object->run();

			lock_module();
			_storage<T>->_instances.deleteNode(node);
			unlock_module();
		} else {
			PX4_ERR("failed to instantiate object");
			ret = -1;
		}

		if (object) {
			delete object;
		}

		return ret;
	}

	/**
	 * @brief Stars the command, ('command start <name>'), checks if name already exists
	 *        and calls T::task_spawn() for new instance if it doesn't.
	 * @param argc The task argument count.
	 * @param argc Pointer to the task argument variable array.
	 * @return Returns 0 if successful, -1 otherwise.
	 */
	static int start_command_base(int argc, char *argv[])
	{
		int ret = 0;

		lock_module();

		// Spawn task, keep 'name' argument because px4getopt skips the first arg
		ret = T::task_spawn(argc, argv);
		if (ret != 0) {
			PX4_ERR("Task spawn failed");
			ret = 1;
		}

		unlock_module();
		return ret;
	}

	/**
	 * @brief Stops the command, ('command stop <name>'), request the module instance <name> to stop
	 *        and waits for the task to complete.
	 * @return Returns 0 if successful, -1 otherwise.
	 */
	static int stop_command(InstanceNode<T> *node)
	{
		int ret = 0;

		// This function must be called with module locked!
		if (!check_locked()) {
			PX4_ERR("stop_command called without module locked!");
			return 1;
		}

		if (!node) {
			PX4_ERR("stop: no instance given");
			return 1;
		}

		T *object = node->_obj.load();
		int task_id = node->_tid;
		if (object) {
			object->request_stop();
			unsigned int i = 0;
			do {
				unlock_module();
				px4_usleep(10000); // 10 ms
				lock_module();
				if (++i > 500 && is_running(object)) { // wait at most 5 sec
					PX4_ERR("timeout, forcing stop");
					px4_task_delete(task_id);

					for (auto t: _storage<T>->_instances) {
						if (t->_obj.load() == object) {
							_storage<T>->_instances.remove(t);
							delete t;
							break;
						}
					}

					ret = -1;
					delete object;
					break;
				}
			} while (is_running(object));
		} else {
			_storage<T>->_instances.remove(node);
			delete node;
		}

		return ret;
	}

	/**
	 * @brief Handle 'command status': check if running and call print_status() if it is
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int status_command(InstanceNode<T> *node)
	{
		// This function must be called with module locked!
		if (!check_locked()) {
			PX4_ERR("stop_command called without module locked!");
			return 1;
		}

		int ret = -1;
		if (!node) {
			PX4_INFO("instance not found");
			return -1;
		}

		T* object = node->_obj.load();
		if (object) {
			PX4_INFO("");
			PX4_INFO("instance '%s':", node->_name);
			PX4_INFO("========");
			ret = object->print_status();
			if (ret < 0) {
				PX4_INFO("    status error");
			}
		} else {
			PX4_ERR("    Instance object missing");
		}

		return ret;
	}

	/**
	 * @brief Print the status if the module is running. This can be overridden by the module to provide
	 * more specific information.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	virtual int print_status()
	{
		PX4_INFO("    running");
		return 0;
	}

	/**
	 * @brief Main loop method for modules running in their own thread. Called from run_trampoline().
	 *        This method must return when should_exit() returns true.
	 */
	virtual void run()
	{
	}

	/**
	 * @brief Returns the status of the module.
	 * @return Returns true if the module is running, false otherwise.
	 */
	static bool is_running(T* obj)
	{
		bool running = false;
		if (_storage<T>) {
			for (auto t: _storage<T>->_instances) {
				if (t->_obj.load() == obj) {
					running = true;
					break;
				}
			}
		}
		return running;
	}

	static bool is_running()
	{
		if (_storage<T> && !_storage<T>->_instances.empty()) {
			return true;
		}
		return false;
	}

protected:

	/**
	 * @brief Tells the module to stop (used from outside or inside the module thread).
	 */
	virtual void request_stop()
	{
		_task_should_exit.store(true);
	}

	/**
	 * @brief Checks if the module should stop (used within the module thread).
	 * @return Returns True iff successful, false otherwise.
	 */
	bool should_exit() const
	{
		return _task_should_exit.load();
	}

	/**
	 * @brief Waits until _object is initialized, (from the new thread). This can be called from task_spawn().
	 *   example: wait_until_running(2000, argc, argv);
	 * @return Returns 0 iff successful, -1 on timeout or otherwise.
	 */
	static int wait_until_running(int argc, char *argv[], int timeout_ms = 1000)
	{
		if (argc > 0) {
			int i = 0;
			char *name = argv[0];
			bool waiting = true;

			do {
				px4_usleep(2000);
				auto node = get_instance_by_name(name);
				if (node) {
					if (node->_obj.load()) {
						waiting = false;
					}
				}
				if (++i >= timeout_ms / 2) {
					waiting = false;
				}
			} while (waiting);

			if (i >= timeout_ms / 2) {
				PX4_ERR("Timed out while waiting for thread to start");
				return -1;
			}
			return 0;
		}

		return 1;
	}

	/**
	 * @brief Get the module's object instance by name, (this is null if it's not running).
	 */
	static InstanceNode<T> *get_instance_by_name(char *name)
	{
		for (auto t: _storage<T>->_instances) {
			if (strncmp(t->_name, name, sizeof(t->_name)) == 0) {
				return t;
			}
		}
		return nullptr;
	}

	/**
	 * @brief Get the module's object instance by task id, (this is null if it's not running).
	 */
	static InstanceNode<T> *get_instance_by_tid(int tid)
	{
		for (auto t: _storage<T>->_instances) {
			if (t->_tid == tid) {
				return t;
			}
		}
		return nullptr;
	}

	// Dummy task_id variable to keep interface backward compatilible
	static int _task_id;

private:

	/**
	 * @brief lock_module Mutex to lock the module thread.
	 */
	static void lock_module()
	{
		pthread_mutex_lock(&_storage<T>->_multi_inst_mod_mutex);
	}

	/**
	 * @brief unlock_module Mutex to unlock the module thread.
	 */
	static void unlock_module()
	{
		pthread_mutex_unlock(&_storage<T>->_multi_inst_mod_mutex);
	}

	static bool check_locked()
	{
		if (pthread_mutex_trylock(&_storage<T>->_multi_inst_mod_mutex) == 0) {
			// Mutex was not locked!
			pthread_mutex_unlock(&_storage<T>->_multi_inst_mod_mutex);
			return false;
		}
		return true;
	}

	/** @var _task_should_exit Boolean flag to indicate if the task should exit. */
	px4::atomic_bool _task_should_exit{false};

};

template<class T>
int MultiInstanceModuleBase<T>::_task_id{-1};

#endif /* __cplusplus */
