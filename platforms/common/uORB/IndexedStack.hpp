#pragma once

/*
 * This is a single-linked list/stack which items can be accessed via handle of
 * type H. The handle can be either an index (int8_t) to a pre-allocated list,
 * or a direct pointer to the item. The items should have a "next" member
 * variable of the intended handle type.
 *
 * TODO: 1) There could be an IndexedStackItem struct from which the actual
 *          could be inherited from
 *       2) There could be an iterator for nicer indexing through the list
*/

template<class T, typename H>
class IndexedStack
{
public:
	IndexedStack(T *pool) : _pool(pool)
	{
		clear_handle(_head);
	}

	void push(H handle)
	{
		if (handle_valid(handle)) {
			T *item = peek(handle);
			item->next = _head;
			_head = handle;
		}
	}

	H pop()
	{
		H ret = _head;

		if (handle_valid(ret)) {
			T *item = head();
			_head = item->next;
		}

		return ret;
	}

	bool rm(H handle)
	{
		H p = _head;
		H r; clear_handle(r);

		if (!handle_valid(handle) ||
		    !handle_valid(p)) {
			return r;
		}

		if (p == handle) {
			// remove the first item
			T *item = head();
			_head = item->next;
			r  = p;

		} else {
			while (handle_valid((r = peek(p)->next))) {
				if (r == handle) {
					T *prev = peek(p);
					T *item = peek(r);
					// remove the item
					prev->next = item->next;
					break;
				}

				p = r;
			}
		}

		return handle_valid(r) ? true : false;
	}

	T *head() {return peek(_head);}
	bool empty() {return !handle_valid(_head);}

	/* Helpers for different handle types */
	T *peek(int8_t handle) { return handle_valid(handle) ? &_pool[handle] : nullptr; }
	T *peek(void *handle) { return static_cast<T *>(handle); }

	static void clear_handle(int8_t &x) { x = -1; };
	static void clear_handle(void *&x) { x = nullptr; };
	static bool handle_valid(int8_t handle) { return handle >= 0; }
	static bool handle_valid(void *handle) { return handle != nullptr; }

private:
	T *_pool;
	H _head;
};
