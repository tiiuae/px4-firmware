/****************************************************************************
 *
 *   Copyright (c) 2023 Technology Innovation Institute. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <malloc.h>
#include <pthread.h>
#include <spawn.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/compiler.h>

#ifdef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_EXEC_ARGS 256

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct main_args_s
{
  main_t     entry;
  int        prio;
  int        argc;
  FAR char  *argv[];
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main_trampoline
 *
 * Description:
 *   Trampoline function for pthread to pass argv among other things
 *
 ****************************************************************************/

static void *main_trampoline(void *ptr)
{
  struct main_args_s *args = (struct main_args_s *) ptr;
  pthread_setschedprio(pthread_self(), args->prio);
  pthread_setname_np(pthread_self(), args->argv[0]);
  args->entry(args->argc, args->argv);
  free(ptr);
  return NULL;
}

/****************************************************************************
 * Name: task_create
 *
 * Description:
 *   Adaptation for CONFIG_BUILD_KERNEL to compile task_create using pthreads
 *   instead of NuttX tasks.
 *
 ****************************************************************************/

int task_create(FAR const char *name, int priority,
                int stack_size, main_t entry, FAR char * const argv[])
{
  FAR struct main_args_s *args;
  FAR char *ptr;
  pthread_t pid;
  pthread_attr_t attr;
  size_t argvsize = 0;
  size_t argssize;
  int argc = 0;
  int ret;
  int i;

  /* Get the number of arguments and the size of the argument list */

  if (argv)
    {
      while (argv[argc])
        {
          argvsize += strlen(argv[argc]) + 1;
          if (argc >= MAX_EXEC_ARGS)
            {
              ret = E2BIG;
              goto update_errno;
            }

          argc++;
        }
    }

  /* Name is a part of argv */

  argvsize += strlen(name) + 1;

  /* Allocate the struct + memory for argv + name */

  argssize = sizeof(struct main_args_s) + (argc + 2) * sizeof(FAR char *);

  args = (struct main_args_s *)malloc(argssize + argvsize);
  if (!args)
    {
      ret = ENOMEM;
      goto update_errno;
    }

  /* Initialize the struct */

  args->entry = entry;
  args->prio = priority;
  args->argc = argc + 1; /* +1 for name */

  /* Copy the name */

  ptr = (char *)args + argssize;
  args->argv[0] = ptr;
  strcpy(ptr, name);
  ptr += strlen(name) + 1;

  /* Copy the argv list */

  for (i = 0; i < argc; i++)
    {
      args->argv[i + 1] = ptr;
      strcpy(ptr, argv[i]);
      ptr += strlen(argv[i]) + 1;
    }

  /* Terminate the argv[] list */

  args->argv[args->argc] = NULL;

  /* Set the worker parameters */

  pthread_attr_init(&attr);
  pthread_attr_setschedpolicy(&attr, SCHED_RR);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setstacksize(&attr, stack_size);

  /* Start the worker */

  ret = pthread_create(&pid, &attr, &main_trampoline, args);
  if (ret != 0)
    {
      printf("ERROR: pthread_create:%d, errno:%s\n", ret, strerror(ret));
      free(args);
      pid = ERROR;
    }

  pthread_attr_destroy(&attr);

update_errno:
  if (ret != 0)
    {
      set_errno(-ret);
      pid = ERROR;
    }

  return (int)pid;
}

/****************************************************************************
 * Name: task_delete
 *
 * Description:
 *   Adaptation for CONFIG_BUILD_KERNEL to compile task_create using pthreads
 *   instead of NuttX tasks.
 *
 ****************************************************************************/

int task_delete(int pid)
{
  int ret;

  if (pid == pthread_self())
    {
      ret = pthread_join(pid, NULL);
      pthread_exit(NULL);
    }
  else
    {
      ret = pthread_cancel(pid);
    }

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

#endif /* CONFIG_BUILD_KERNEL */
