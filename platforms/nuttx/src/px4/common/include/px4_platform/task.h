/****************************************************************************
 * /px4-firmware/platforms/nuttx/src/px4/common/include/px4_platform/task.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#pragma once

#include <nuttx/config.h>

__BEGIN_DECLS

#ifdef CONFIG_BUILD_KERNEL
int task_create(FAR const char *name, int priority,
                int stack_size, main_t entry, FAR char * const argv[]);
int task_delete(int pid);
#endif

__END_DECLS
