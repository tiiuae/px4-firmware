/****************************************************************************
 * boards/risc-v/icicle/mpfs/scripts/ld.script
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

MEMORY
{
  progmem      (r) : ORIGIN = 0xAFB00000, LENGTH = 5M
}

OUTPUT_ARCH("riscv")

ENTRY(_app_start)

EXTERN(_main_toc)

SECTIONS
{
    .firmware : {
        _fw_start = ABSOLUTE(.);

        /*
        This signature provides the bootloader with a way to delay booting
        */
        _bootdelay_signature = ABSOLUTE(.);
        FILL(0x01ecc2925d7d05c5)
        . += 8;

        /* The ToC itself */
        *(.main_toc)
        . = 0x1000;

        /* The application firmware payload itself */
        _app_start = ABSOLUTE(.);
        *(.firmware)
        _app_end = ABSOLUTE(.);

        /* Start of the image signature. This
         *  has to be in the end of the image
         */
        _boot_signature = ALIGN(4);
        _fw_end = ABSOLUTE(.);
    } > progmem
}
