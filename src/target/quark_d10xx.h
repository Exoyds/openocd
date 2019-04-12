/*
 * Copyright(c) 2014-2015 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Contact Information:
 * Intel Corporation
 */

#ifndef QUARK_D10XX_H
#define QUARK_D10XX_H
#include <helper/types.h>
#include "register.h"

static uint32_t const DEBUG_MEMORY_ACCESS_CMD           = 0x304;
static uint32_t const DEBUG_MEMORY_ACCESS_CMD_START     = 0x2;
static uint32_t const DEBUG_MEMORY_ACCESS_CMD_WRITE     = 0x4;
static uint32_t const DEBUG_MEMORY_ACCESS_CMD_SEQ       = 0x108;

static uint32_t const DEBUG_MEMORY_ACCESS_CMD_ALL_BYTES = 0xF0;

static uint32_t const DEBUG_MEMORY_ACCESS_COUNT         = 0x314;

static uint32_t const DEBUG_MEMORY_ACCESS_ADDRESS       = 0x308;
static uint32_t const DEBUG_MEMORY_ACCESS_WR_DATA       = 0x30C;
static uint32_t const DEBUG_MEMORY_ACCESS_RD_DATA       = 0x310;

static uint32_t const DEBUG_REGISTERS_ACCESS_BASE       = 0x200;

static uint32_t const DEBUG_CONTROL_STATUS              = 0x100;
static uint32_t const DEBUG_CONTROL_STATUS_HALTED       = 0x40000000;
static uint32_t const DEBUG_CONTROL_STATUS_IRQ_DISABLE  = 0x00020000;

static uint32_t const DEBUG_CONTROL_COMMAND             = 0x104;
static uint32_t const DEBUG_CONTROL_COMMAND_HALT        = 0x80000000;
static uint32_t const DEBUG_CONTROL_COMMAND_STEP        = 0x04000000;
static uint32_t const DEBUG_CONTROL_COMMAND_RESUME      = 0x40000000;

static uint32_t const DEBUG_CONTROL_BREAK               = 0x10C;
static uint32_t const DEBUG_CONTROL_RSTOFBE             = 0x80000000;

static uint32_t const DEBUG_CONTROL_HALT                = 0x110;
static uint32_t const DEBUG_CONTROL_HALT_INT3           = 0x01000000;
static uint32_t const DEBUG_CONTROL_HALT_RST            = 0x80000000;
static uint32_t const DEBUG_CONTROL_PWR_RST             = 0x114;
static uint32_t const DEBUG_CONTROL_PWR_RST_HRESET      = 0x00000004;

static uint32_t const DEBUG_DEVID                       = 0x0;
static uint32_t const DEBUG_DEVID_VALUE                 = 0x8086DEBC;

//FLASH registers
static uint32_t const QUARK_D_INST_ROM_BASE             = 0x00000000;
static uint32_t const QUARK_D_INST_ROM_FLAG             = 0x0;
static uint32_t const QUARK_D_INST_FLASH_BASE           = 0x20000000;
static uint32_t const QUARK_D_INST_FLASH_FLAG           = 0x2;
static uint32_t const QUARK_D_DATA_FLASH_BASE           = 0x40000000;
static uint32_t const QUARK_D_DATA_FLASH_FLAG           = 0x4;
static uint32_t const QUARK_D_MEM_SELECT_MASK           = 0x0FFFFFFF;

static uint32_t const QUARK_D_FLASH_INSTR_WR_ADDR_REG   = 0x8000004C;
static uint32_t const QUARK_D_FLASH_INSTR_WR_DATA_0_REG = 0x80000050;
static uint32_t const QUARK_D_FLASH_INSTR_WR_DATA_1_REG = 0x80000054;
static uint32_t const QUARK_D_FLASH_ERASE_WRITE_CNTL_REG= 0x80000044;
static uint32_t const QUARK_D_FLASH_STATUS_REG          = 0x80000048;
static uint32_t const QUARK_D_FLASH_DATA_WR_ADDR_REG    = 0x80000060;
static uint32_t const QUARK_D_FLASH_DATA_WR_DATA_REG    = 0x80000064;

static uint32_t const QUARK_D_FLASH_ERASE_TCK_TIME      = 128500;
static uint32_t const QUARK_D_FLASH_PROG_TCK_TIME       = 270;


//TAP registers
static uint32_t const TAP_CMD                           = 4;
static uint32_t const TAP_CMD_LEN                       = 3;
static uint32_t const TAP_CMD_RD                        = 0;
static uint32_t const TAP_CMD_WR                        = 1;

static uint32_t const TAP_ADDR                          = 5;
static uint32_t const TAP_ADDR_LEN                      = 12;

static uint32_t const TAP_WR                            = 7;
static uint32_t const TAP_WR_LEN                        = 32;

static uint32_t const TAP_RD                            = 8;
static uint32_t const TAP_RD_LEN                        = 32;

static uint32_t const TAP_START                         = 9;
static uint32_t const TAP_START_LEN                     = 2;
static uint32_t const TAP_START_CMD                     = 1;

static uint32_t const TAP_STREAM                        = 0xA;
static uint32_t const TAP_STREAM_LEN                    = 34;

struct quark_d_arch
{
    struct reg_cache *core_cache;
};

struct quark_d_register
{
    uint32_t id;
    struct target* target;
};

#endif

