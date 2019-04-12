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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <jtag/jtag.h>
#include "quark_d10xx.h"

#include "target.h"
#include "target_type.h"
#include "breakpoints.h"

static struct {
    uint32_t mem[512];
    uint32_t mem_o[512];
    int dirty;
} quark_d_data_flash[2];

static struct {
    uint32_t mem[1024];
    uint32_t mem_o[1024];
    int dirty;
} quark_d_inst_flash[8];


typedef enum {E_INST, E_DATA} quark_d_flash_type;

static int if_memory_writes;
static int inside_macro_transaction = 0;

typedef enum {RW_INSTR = 0, RW_WRITE = 1, RW_ACCESS = 3} hw_type_rw;
typedef enum {LEN_LEN1 = 0, LEN_LEN2 = 1, LEN_LEN4 = 3} hw_type_len;

struct {
    unsigned used;
    unsigned address;
    hw_type_rw rw;
    hw_type_len len;
} hw_bps[4];

int quark_d_poll(struct target *target);
int quark_d_target_create(struct target *target, Jim_Interp *interp);
int quark_d_arch_state(struct target *target);
int quark_d_init_target(struct command_context *cmd_ctx, struct target *target);

int quark_d_halt(struct target *target);
int quark_d_resume(struct target *target, int current, target_addr_t address, int handle_breakpoints, int debug_execution);
int quark_d_step(struct target *target, int current, target_addr_t address, int handle_breakpoints);

int quark_d_assert_reset(struct target *target);
int quark_d_deassert_reset(struct target *target);
int quark_d_soft_reset_halt(struct target *target);

int quark_d_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size, enum target_register_class reg_class);

int quark_d_read_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer);
int quark_d_write_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer);

int quark_d_examine(struct target *target);
int quark_d_bulk_write_memory(struct target *target,
        uint32_t address, uint32_t count, const uint8_t *buffer);

int quark_d_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int quark_d_remove_breakpoint(struct target *target, struct breakpoint *breakpoint);

int quark_d_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int quark_d_remove_watchpoint(struct target *target, struct watchpoint *watchpoint);


//Low level access to debug controller
int
quark_d_debug_write_register(struct target * target, uint32_t const address, uint32_t const data);
int
quark_d_debug_read_register(struct target * target, uint32_t const address, uint32_t *const data);

int
quark_d_read_mem_word(struct target * target, uint32_t const address, uint32_t* const data);
int
quark_d_write_mem_word(struct target * target, uint32_t const address, uint32_t const data);

static int 
quark_d_df_erase(struct target* target, uint32_t page_number);
static int 
quark_d_df_prog(struct target* target, uint32_t address, uint32_t data);
static int 
quark_d_cf_erase(struct target* target, uint32_t page_number);
static int 
quark_d_cf_prog(struct target* target, uint32_t address, uint32_t data0, uint32_t data1);

int
quark_d_read_flash_shadow_copy(struct target* target, quark_d_flash_type flash_type, uint32_t page_number);
int
quark_d_write_flash_shadow_copy(struct target* target);

int
quark_d_save_context(struct target * target);
int
quark_d_restore_context(struct target*  target);

int
quark_d_write_mem_chunk(
    struct target * target,
    uint32_t const address,
    uint32_t * const data,
    uint32_t const count);

int
quark_d_jtag_set_instruction(struct target* target, int new_instr)
{
    struct jtag_tap *tap;

    tap = target->tap;
    if (tap == NULL)
        return ERROR_FAIL;

    if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != (uint32_t)new_instr)
    {
        struct scan_field field;
        uint32_t t = new_instr;

        field.num_bits = tap->ir_length;
        field.out_value = (void*)&t;
        field.in_value = NULL;

        jtag_add_ir_scan(tap, &field, TAP_IDLE);
    }

    return ERROR_OK;
}

struct target_type quark_d10xx_target =
{
    .name = "quark_d10xx",

    .poll = quark_d_poll,
    .arch_state = quark_d_arch_state,

    .target_request_data = NULL,

    .halt = quark_d_halt,
    .resume = quark_d_resume,
    .step = quark_d_step,

    .assert_reset = quark_d_assert_reset,
    .deassert_reset = quark_d_deassert_reset,
    .soft_reset_halt = quark_d_soft_reset_halt,

    .get_gdb_reg_list = quark_d_get_gdb_reg_list,

    .read_memory = quark_d_read_memory,
    .write_memory = quark_d_write_memory,
    //.bulk_write_memory = mips_m4k_bulk_write_memory,
    //.checksum_memory = mips_m4k_checksum_memory,
    //.blank_check_memory = NULL,

    //.run_algorithm = mips32_run_algorithm,

    .add_breakpoint = quark_d_add_breakpoint,
    .remove_breakpoint = quark_d_remove_breakpoint,
    .add_watchpoint = quark_d_add_watchpoint,
    .remove_watchpoint = quark_d_remove_watchpoint,
    //.bulk_write_memory = quark_d_bulk_write_memory,

    .target_create = quark_d_target_create,
    .init_target = quark_d_init_target,
    .examine = quark_d_examine,
};

const unsigned quark_d_registers_number = 41;
char* quark_d_core_reg_list[] =
{
    "eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi",
    "eip", "eflags", "cs", "ss", "ds", "es", "fs", "gs", 
    "st0", "st1", "st2", "st3", "st4", "st5", "st6", "st7",
    "fctrl", "fstat", "ftag", "fcs", "fcoff", "fds", "fdoff", "fop",
    "xmm0", "xmm1", "xmm2", "xmm3", "xmm4", "xmm5", "xmm6", "xmm7", "mxcsr"
};

int
quark_d_set_core_reg(struct reg *reg, uint8_t *buf)
{
    struct quark_d_register* nc_reg = (struct quark_d_register*)reg->arch_info;
    if (nc_reg->id < 10) //some i386 registers do not exits in NC
    {
        //LOG_INFO("updating cache for %s register", reg->name);
    }
    else
    {
        return ERROR_OK;
    }
    uint32_t value = buf_get_u32(buf, 0, 32);
    
    if (nc_reg->target->state != TARGET_HALTED)
    {
        return ERROR_TARGET_NOT_HALTED;
    }
    buf_set_u32(reg->value, 0, 32, value);
    reg->dirty = 1;
    reg->valid = 1;
    
    return ERROR_OK;
}

int
quark_d_get_core_reg(struct reg *reg)
{
    //LOG_WARNING("quark_d_get_core_reg IS NOT IMPLEMENTED");
    return ERROR_OK;
}

static const struct reg_arch_type quark_d_reg_type = {
    .get = quark_d_get_core_reg,
    .set = quark_d_set_core_reg,
};


int
quark_d_target_create(struct target *target, Jim_Interp *interp)
{
    unsigned i = 0;
    const unsigned num_regs = quark_d_registers_number;
    //struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
    
    struct quark_d_arch *arch_info = calloc(1, sizeof(struct quark_d_arch));
    if(arch_info == NULL) {
        LOG_ERROR("%s Could not allocate arch_info", __func__);
        return ERROR_FAIL;
    }

    struct reg *reg_list = malloc(sizeof(struct reg) * num_regs);
    if(reg_list == NULL) {
        LOG_ERROR("%s Could not allocate reg_list", __func__);
        return ERROR_FAIL;
    }

    if_memory_writes = 0;

    target->arch_info = arch_info;

    //reg cache initialization
    arch_info->core_cache = malloc(sizeof(struct reg_cache));
    if(arch_info->core_cache == NULL) {
        LOG_ERROR("%s Could not allocate core cache", __func__);
        return ERROR_FAIL;
    }
    
    arch_info->core_cache->name = "quark_d registers";
    arch_info->core_cache->next = NULL;
    arch_info->core_cache->reg_list = reg_list;
    arch_info->core_cache->num_regs = num_regs;
    
    for (i = 0; i < num_regs; i++) {
        reg_list[i].name = quark_d_core_reg_list[i];
        
        if (i < 16) {          //GPR
            reg_list[i].size = 32;
            reg_list[i].value = calloc(1, 4);
            *((int*)reg_list[i].value) = 0x100 + i;
        } else if (i < 24) {   //FPU data
            reg_list[i].size = 80;
            reg_list[i].value = calloc(1, 8);
            *((float*)reg_list[i].value) = i + (float)0.0;
        } else if (i < 27) {   //FPU ctrl
            reg_list[i].size = 16;
            reg_list[i].value = calloc(1, 2);
            *((short*)reg_list[i].value) = 0;
        } else if (i == 27) {  // FPU FCS
            reg_list[i].size = 16;
            reg_list[i].value = calloc(1, 2);
            *((short*)reg_list[i].value) = 0;
        } else if (i == 28) {  // FPU FCOFF
            reg_list[i].size = 32;
            reg_list[i].value = calloc(1, 4);
            *((int*)reg_list[i].value) = 0;
        } else if (i == 29) {  // FPU FDS
            reg_list[i].size = 16;
            reg_list[i].value = calloc(1, 2);
            *((short*)reg_list[i].value) = 0;
        } else if (i == 30) {  // FPU FDOFF
            reg_list[i].size = 32;
            reg_list[i].value = calloc(1, 4);
            *((int*)reg_list[i].value) = 0;
        } else if (i == 31) {  // FPU FOP
            reg_list[i].size = 11;
            reg_list[i].value = calloc(1, 2);
            *((short*)reg_list[i].value) = 0;
        } else if (i < 40) {   // XMM data
            reg_list[i].size = 128;
            reg_list[i].value = calloc(1, 16);
            *((double*)reg_list[i].value) = i + (double)0.0;
        } else {               // XMM ctrl
            reg_list[i].size = 32;
            reg_list[i].value = calloc(1, 4);
            *((int*)reg_list[i].value) = 0;
        }
  
        reg_list[i].dirty = 0;
        reg_list[i].valid = 0;
        reg_list[i].type = &quark_d_reg_type;
        reg_list[i].arch_info = malloc(sizeof(struct quark_d_register));
        ((struct quark_d_register*)reg_list[i].arch_info)->id = i;
        ((struct quark_d_register*)reg_list[i].arch_info)->target = target;
    }
    
    for (i = 0; i < 2; i++) {
        quark_d_data_flash[i].dirty = 0;
    }
    
    for (i = 0; i < 8; i++) {
        quark_d_inst_flash[i].dirty = 0;
    }
    
    return ERROR_OK;
}

int quark_d_poll_requested = 0;

static void
macro_transaction_begin(void)
{
    inside_macro_transaction = 1;
}

static int
macro_transaction_end(void)
{
    inside_macro_transaction = 0;
    if (jtag_execute_queue() != ERROR_OK)
    {
        LOG_ERROR("macro transaction failed");
        return ERROR_FAIL;
    }
    return ERROR_OK;
}

int
quark_d_poll(struct target *target)
{
    int retval = 0;
    uint32_t nc_state = 0;
    unsigned dbg_reason, tmp_dbg_reason;
    unsigned dbg_status;
    int i;
    unsigned devid[2];
    dbg_reason = DBG_REASON_DBGRQ;

    macro_transaction_begin();
      quark_d_debug_read_register(target, DEBUG_DEVID, &devid[0]); 
      quark_d_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state);
      quark_d_debug_read_register(target, DEBUG_DEVID, &devid[1]); 
    macro_transaction_end();

    if ((devid[0] == DEBUG_DEVID_VALUE) && (devid[1] == DEBUG_DEVID_VALUE) && (nc_state & DEBUG_CONTROL_STATUS_HALTED))
    {
        target->state = TARGET_HALTED;
        if (quark_d_poll_requested)
        {
            if (quark_d_poll_requested == DBG_REASON_SINGLESTEP)
            {
                //enable interrupts
                quark_d_debug_write_register(target, DEBUG_CONTROL_STATUS, 0x0);
            }
            
            if ((retval = quark_d_debug_read_register(target, 0x398, &dbg_status)) != ERROR_OK)
            {
                return retval;
            }

            for (i = 0; i < 4; i++) {
                if (dbg_status & 0x01) {
                    if (hw_bps[i].rw == RW_INSTR) {
                        tmp_dbg_reason = DBG_REASON_BREAKPOINT;
                    }
                    else {
                        tmp_dbg_reason = DBG_REASON_WATCHPOINT;
                    }
                    
                    if (dbg_reason != DBG_REASON_DBGRQ) {
                        dbg_reason = DBG_REASON_WPTANDBKPT;
                    } else {
                        dbg_reason = tmp_dbg_reason;
                    }
                }
                dbg_status = dbg_status >> 1;
            }
            
            target->debug_reason = dbg_reason;
            target_call_event_callbacks(target, TARGET_EVENT_HALTED);
            quark_d_poll_requested = 0;
            quark_d_save_context(target); //update reg cache
        }
        else
        {
            target->debug_reason = DBG_REASON_DBGRQ;
        }
    }
    //LOG_INFO("quark_d_poll");
    return ERROR_OK;
}

int
quark_d_arch_state(struct target *target)
{
    return ERROR_OK;
}

int
quark_d_halt(struct target *target)
{
    int retval = 0;
    uint32_t nc_state = 0;
    //check NC state
    if ((retval = quark_d_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) != ERROR_OK)
    {
        return retval;
    }
    if (nc_state & DEBUG_CONTROL_STATUS_HALTED)
    {
        LOG_ERROR("Halt request when quark_d is already in halted state");
    }
    //issue Halt command
    if ((retval = quark_d_debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_HALT)) != ERROR_OK)
    {
        return retval;
    }
    //issue error if we are still running
    if ((retval = quark_d_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state)) != ERROR_OK)
    {
        return retval;
    }
    if (!(nc_state & DEBUG_CONTROL_STATUS_HALTED))
    {
        LOG_ERROR("NC is not halted after Halt command");
    }
    if (quark_d_poll_requested)
    {
        //quark_d_poll_requested = 0;
    }
    target->state = TARGET_HALTED;
    target->debug_reason = DBG_REASON_DBGRQ;
    return ERROR_OK;
}

int
quark_d_resume(struct target *target, int current, target_addr_t address, int handle_breakpoints, int debug_execution)
{
    quark_d_write_flash_shadow_copy(target);
    quark_d_restore_context(target); //upload reg values into HW
    quark_d_debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_RESUME);
    //poll state <- don't do this
    //target_call_event_callbacks(target, TARGET_EVENT_HALTED);
    quark_d_poll_requested = DBG_REASON_BREAKPOINT;
    target->state = TARGET_RUNNING;
    //quark_d_save_context(target); //update reg cache
    return ERROR_OK;
}

int
quark_d_step(struct target *target, int current, target_addr_t address, int handle_breakpoints)
{   
    //LOG_INFO("quark_d_step");
    quark_d_write_flash_shadow_copy(target);
    quark_d_restore_context(target); //upload reg values into HW
    quark_d_debug_write_register(target, DEBUG_CONTROL_STATUS, DEBUG_CONTROL_STATUS_IRQ_DISABLE);
    quark_d_debug_write_register(target, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_STEP);
    //poll state <- don't do this
    //target_call_event_callbacks(target, TARGET_EVENT_HALTED);
    quark_d_poll_requested = DBG_REASON_SINGLESTEP;
    target->state = TARGET_RUNNING;
    //quark_d_save_context(target); //update reg cache
    
    return ERROR_OK;
}

int
quark_d_assert_reset(struct target *target)
{   
    //LOG_INFO("quark_d_assert_reset");
    quark_d_write_flash_shadow_copy(target);
    int retval = 0;
    if ((retval = quark_d_debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK)
    {
        return retval;
    }
    
    return ERROR_OK;
}

int
quark_d_deassert_reset(struct target *target)
{   
    int retval = 0;
    //LOG_INFO("quark_d_deassert_reset");
    quark_d_write_flash_shadow_copy(target);
    if ((retval = quark_d_debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0)) != ERROR_OK)
    {
        return retval;
    }
    
    return ERROR_OK;
}

int
quark_d_soft_reset_halt(struct target *target)
{
    int retval = 0;
    //LOG_INFO("quark_d_soft_reset_halt");
    quark_d_write_flash_shadow_copy(target);
    //assert
    if ((retval = quark_d_debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET)) != ERROR_OK)
    {
        return retval;
    }
    //...and deassert reset
    if ((retval = quark_d_debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0)) != ERROR_OK)
    {
        return retval;
    }
    
    return ERROR_OK;
}

int
quark_d_bulk_write_memory(struct target *target,
        uint32_t address, uint32_t count, const uint8_t *buffer)
{
    return quark_d_write_memory(target, address, 4, count, buffer);
}

//gdb_server expects valid reg values and will use set method for updating reg values
int
quark_d_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size, enum target_register_class reg_class)
{
    unsigned i;
    struct quark_d_arch *arch_info = (struct quark_d_arch *)target->arch_info;
    
    *reg_list = malloc(sizeof(struct reg*) * quark_d_registers_number);
    if(*reg_list == NULL) {
        LOG_ERROR("%s Could not allocate reg_list", __func__);
        return ERROR_FAIL;
    }

    
    for (i = 0; i < quark_d_registers_number; ++i)
    {
        (*reg_list)[i] = &arch_info->core_cache->reg_list[i];
    }
    *reg_list_size = quark_d_registers_number;
    return ERROR_OK;
}

int
quark_d_read_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
    unsigned i = 0; //byte count
    uint32_t x = 0; //buffer
    int retval = 0;
    unsigned const buffer_size = count * size;
    unsigned word_start_offset = 0;
    if (address & 0x3) //Address is not aligned
    {
        if ((retval = quark_d_read_mem_word(target, address & (~0x3), &x)) != ERROR_OK)
        {
            return retval;
        }
        while ((address + i) & 0x3 && i != buffer_size)
        {
            *(buffer + i) = ((uint8_t*)(&x))[(address + i) & 0x3];
            ++i;
        }
    }
    for (;i + 4 <= buffer_size; i += 4)
    {
        if ((retval = quark_d_read_mem_word(target, address + i, (uint32_t*)(buffer + i))) != ERROR_OK)
        {
            return retval;
        }
    }
    if (buffer_size == i) return ERROR_OK;
    if ((retval = quark_d_read_mem_word(target, address + i, &x)) != ERROR_OK)
    {
        return retval;
    }
    word_start_offset = i;
    while (i != buffer_size)
    {
        *(buffer + i) = ((uint8_t*)(&x))[(i - word_start_offset) & 0x3]; 
        ++i;
    }
    return ERROR_OK;
}

int
quark_d_write_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer)
{
    unsigned i = 0; //byte count
    uint32_t x = 0; //buffer
    int retval = 0;
    unsigned word_start_offset = 0;
    unsigned const buffer_size = count * size;
    if (address & 0x3) //Address is not aligned
    {
        if ((retval = quark_d_read_mem_word(target, address & (~0x3), &x)) != ERROR_OK)
        {
            return retval;
        }
        while ((address + i) & 0x3 && i != buffer_size)
        {
            ((uint8_t*)(&x))[(address + i) & 0x3] = *(buffer + i);
            ++i;
        }
        if ((retval = quark_d_write_mem_word(target, address & (~0x3), x)) != ERROR_OK)
        {
            return retval;
        }
    }
    uint32_t chunk_write_size = (buffer_size - i) & ~0x3;
    if (chunk_write_size < 8)
    {
        for (; i + 4 <= buffer_size; i += 4)
        {
            if ((retval = quark_d_write_mem_word(target, address + i, *(uint32_t*)(buffer + i))) != ERROR_OK)
            {
                return retval;
            }
        }
    }
    else
    {
        quark_d_write_mem_chunk(target, address + i, (uint32_t*)(buffer + i), chunk_write_size / 4);
        i += chunk_write_size;
    }
    if (buffer_size == i) return ERROR_OK;
    if ((retval = quark_d_read_mem_word(target, address + i, &x)) != ERROR_OK)
    {
        return retval;
    }
    word_start_offset = i;
    while (i != buffer_size)
    {
        ((uint8_t*)(&x))[(i - word_start_offset) & 0x3] = *(buffer + i);
        ++i;
    }
    if ((retval = quark_d_write_mem_word(target, (address + i) & ~0x3, x)) != ERROR_OK)
    {
        return retval;
    }
    return ERROR_OK;
}

int quark_d_init_target(struct command_context *cmd_ctx, struct target *target)
{
    return ERROR_OK;
}

int quark_d_examine(struct target *target)
{
    //initialize register values
#if 0
    uint32_t id;

    quark_d_debug_read_register(target, 0, &id);
    if (id != 0x8086DEBC)
    {
        LOG_ERROR("Id register has wrong value %08X", id);
        return ERROR_FAIL;
    }
#endif
    quark_d_debug_write_register(target, DEBUG_CONTROL_PWR_RST, DEBUG_CONTROL_PWR_RST_HRESET);
    quark_d_debug_write_register(target, DEBUG_CONTROL_BREAK, DEBUG_CONTROL_RSTOFBE);
    quark_d_debug_write_register(target, DEBUG_CONTROL_HALT, DEBUG_CONTROL_HALT_INT3);
    quark_d_debug_write_register(target, DEBUG_CONTROL_PWR_RST, 0);
    target->state = TARGET_HALTED;
    quark_d_save_context(target);
    target_set_examined(target);
    return ERROR_OK;
}

int
quark_d_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
    int i               = 0;
    int retval          = 0;
    unsigned hw_ctrl    = 0;
    int bp              = 0;

    if (breakpoint->set)
	{
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}
    
    if (breakpoint->length != 1)
    {
        return ERROR_FAIL;
    }
    else if (breakpoint->type == BKPT_HARD)
    {
        //Find unused HW breakpoint
        //TODO: Maybe it is useful to check hardware state here(at least once)
        for (i = 0; i < 4; ++i)
        {
            if (!hw_bps[i].used)
            {
                bp = i;
                break;
            }
        }
        if (4 == i) //All HW breakpoint are occupied
        {
            return ERROR_FAIL;
        }
        hw_bps[bp].used = 1;
        hw_bps[bp].rw   = RW_INSTR;
        hw_bps[bp].len  = LEN_LEN1;
        hw_bps[bp].address = breakpoint->address;
        for (i = 0; i < 4; ++i)
        {
            if (hw_bps[i].used)
            {
                hw_ctrl |= 1                      << (i * 2 + 1);
                hw_ctrl |= (hw_bps[i].rw & 0x03)  << (i * 4 + 16);
                hw_ctrl |= (hw_bps[i].len & 0x03) << (i * 4 + 18);
            }
        }
        macro_transaction_begin();
            quark_d_debug_write_register(target, 0x39C, hw_ctrl);
            quark_d_debug_write_register(target, 0x380 + bp * 4, breakpoint->address);
        macro_transaction_end();
        //LOG_INFO("Hardware instruction breakpoint at address %X", breakpoint->address);
    }
    else
    {
        if ((retval = target_read_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr)) != ERROR_OK)
        {
            return retval;
        }
        if ((retval = target_write_u8(target, breakpoint->address, 0xCC)) != ERROR_OK)
        {
            return retval;
        }
    }
    breakpoint->set = 1;
    return ERROR_OK;
}

int
quark_d_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
    int i               = 0;
    int retval          = 0;
    int bp              = 0;
    unsigned hw_ctrl    = 0;
    
    if (!breakpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}
    
    if (breakpoint->length != 1)
    {
        return ERROR_FAIL;
    }
    else if (breakpoint->type == BKPT_HARD)
    {
        for (i = 0; i < 4; ++i)
        {
            if ((hw_bps[i].address == breakpoint->address)  &&
                (hw_bps[i].rw      == RW_INSTR)             &&
                (hw_bps[i].len     == LEN_LEN1))
            {
                break;
            }
        }
        if (4 == i) //The breakpoint requested for removal is not found
        {
            return ERROR_FAIL;
        }
        bp = i;
        hw_bps[bp].used = 0;
        for (i = 0; i < 4; ++i)
        {
            if (hw_bps[i].used)
            {
                hw_ctrl |= 1                      << (i * 2 + 1);
                hw_ctrl |= (hw_bps[i].rw & 0x03)  << (i * 4 + 16);
                hw_ctrl |= (hw_bps[i].len & 0x03) << (i * 4 + 18);
            }
        }
        if ((retval = quark_d_debug_write_register(target, 0x39C, hw_ctrl)) != ERROR_OK)
        {
            hw_bps[bp].used = 1;
            return retval;
        }
        //LOG_INFO("Hardware instruction breakpoint is removed: %X", breakpoint->address);
    }
    else
    {
        if ((retval = target_write_u8(target, breakpoint->address, *breakpoint->orig_instr)) != ERROR_OK)
        {
            return retval;
        }
    }
    breakpoint->set = 0;
    return ERROR_OK;
}

int
quark_d_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
    int i               = 0;
    int retval          = 0;
    unsigned hw_ctrl    = 0;
    int bp              = 0;
    hw_type_rw rw       = RW_WRITE;
    hw_type_len len     = LEN_LEN1;
    if (watchpoint->set)
	{
		LOG_WARNING("watchpoint already set");
		return ERROR_OK;
	}
    
    for (i = 0; i < 4; ++i)
    {
        if (!hw_bps[i].used)
        {
            bp = i;
            break;
        }
    }
    if (4 == i) //All HW breakpoint are occupied
    {
        return ERROR_FAIL;
    }
    
    switch (watchpoint->rw)
    {
        case WPT_WRITE:
            rw = RW_WRITE;
            break;
        case WPT_ACCESS:
            rw = RW_ACCESS;
            break;
        default:
            LOG_ERROR("BUG: watchpoint->rw neither write nor access!");
            return ERROR_FAIL;
    }
    
    switch (watchpoint->length) 
    {
        case 1:
            len = LEN_LEN1;
            break;
        case 2:
            len = LEN_LEN2;
            break;
        case 4:
            len = LEN_LEN4;
            break;
        default:
            LOG_ERROR("BUG: watchpoint->length %d is not supported!", watchpoint->length);
            return ERROR_FAIL;
    }
    
    hw_bps[bp].used = 1;
    hw_bps[bp].rw   = rw;
    hw_bps[bp].len  = len;
    hw_bps[bp].address = watchpoint->address;
    for (i = 0; i < 4; ++i)
    {
        if (hw_bps[i].used)
        {
            hw_ctrl |= 1                      << (i * 2 + 1);
            hw_ctrl |= (hw_bps[i].rw & 0x03)  << (i * 4 + 16);
            hw_ctrl |= (hw_bps[i].len & 0x03) << (i * 4 + 18);
        }
    }
    if ((retval = quark_d_debug_write_register(target, 0x39C, hw_ctrl)) != ERROR_OK)
    {
        return retval;
    }
    if ((retval = quark_d_debug_write_register(target, 0x380 + bp * 4, watchpoint->address)) != ERROR_OK)
    {
        return retval;
    }
    //LOG_INFO("Hardware watchpoint set at address %X type %02X length %02X", watchpoint->address, hw_bps[bp].rw, hw_bps[bp].len);
    
    watchpoint->set = 1;
    return ERROR_OK;
}

int
quark_d_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
    int i               = 0;
    int retval          = 0;
    unsigned hw_ctrl    = 0;
    int bp              = 0;
    hw_type_rw rw       = RW_WRITE;
    hw_type_len len     = LEN_LEN1;

    if (!watchpoint->set)
	{
		LOG_WARNING("watchpoint not set");
		return ERROR_OK;
	}
    
    switch (watchpoint->rw)
    {
        case WPT_WRITE:
            rw = RW_WRITE;
            break;
        case WPT_ACCESS:
            rw = RW_ACCESS;
            break;
        default:
            LOG_ERROR("BUG: watchpoint->rw neither write nor access!");
            return ERROR_FAIL;
    }
    
    switch (watchpoint->length) 
    {
        case 1:
            len = LEN_LEN1;
            break;
        case 2:
            len = LEN_LEN2;
            break;
        case 4:
            len = LEN_LEN4;
            break;
        default:
            LOG_ERROR("BUG: watchpoint->length %d is not supported!", watchpoint->length);
            return ERROR_FAIL;
    }
    
    for (i = 0; i < 4; ++i)
    {
        if ((hw_bps[i].address == watchpoint->address)  &&
            (hw_bps[i].rw      == rw)                   &&
            (hw_bps[i].len     == len))
        {
            break;
        }
    }
    if (4 == i) //The breakpoint requested for removal is not found
    {
        return ERROR_FAIL;
    }
    bp = i;
    hw_bps[bp].used = 0;
    for (i = 0; i < 4; ++i)
    {
        if (hw_bps[i].used)
        {
            hw_ctrl |= 1                      << (i * 2 + 1);
            hw_ctrl |= (hw_bps[i].rw & 0x03)  << (i * 4 + 16);
            hw_ctrl |= (hw_bps[i].len & 0x03) << (i * 4 + 18);
        }
    }
    if ((retval = quark_d_debug_write_register(target, 0x39C, hw_ctrl)) != ERROR_OK)
    {
        hw_bps[bp].used = 1;
        return retval;
    }
    //LOG_INFO("Hardware watchpoint is removed at address %X type %02X length %02X", watchpoint->address, hw_bps[bp].rw, hw_bps[bp].len);
    
    watchpoint->set = 0;
    return ERROR_OK;
}

int
quark_d_read_mem_word(
    struct target * target, 
    uint32_t const address, 
    uint32_t* const data) {
    
    int retval = 0;
    //LOG_INFO("Read addr: %08X", address);
    uint32_t flash_addr = (address & QUARK_D_MEM_SELECT_MASK);
    if ((address >> 28) == QUARK_D_INST_FLASH_FLAG) {
        uint32_t flash_page = (flash_addr >> 12) & 0x7;
        if (quark_d_inst_flash[flash_page].dirty) { 
            uint32_t flash_offset = (flash_addr / 4) % 1024;
            *data = quark_d_inst_flash[flash_page].mem[flash_offset];
            //LOG_INFO("IF addr: %08X", address);
            return ERROR_OK;
        }
    } else if ((address >> 28) == QUARK_D_DATA_FLASH_FLAG) {
        uint32_t flash_page = (flash_addr >> 11) & 0x1;
        if (quark_d_data_flash[flash_page].dirty) {
            uint32_t flash_offset = (flash_addr / 4) % 512;
            *data = quark_d_data_flash[flash_page].mem[flash_offset];
            //LOG_INFO("DF addr: %08X", address);
            return ERROR_OK;
        }
    } else {   
    }
    
    //original NC routines
    if ((retval = quark_d_debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK)
    {
        return retval;
    }
    if ((retval = quark_d_debug_write_register(target, DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START)) != ERROR_OK)
    {
        return retval;
    }
    
    if ((retval = quark_d_debug_read_register(target, DEBUG_MEMORY_ACCESS_RD_DATA, data)) != ERROR_OK)
    {
        return retval;
    }
    //LOG_INFO("MR A %08X D %08X", address, *data);
    return ERROR_OK;    
}

int
quark_d_write_mem_chunk(
    struct target * target,
    uint32_t const address,
    uint32_t * const data,
    uint32_t const count)
{
    struct scan_field field;
    uint32_t out_data[2];
    int retval = 0;

    //LOG_INFO("chunk write A: %08X C: %d", address, count);

    if (count > 0xFFFF) {
        LOG_ERROR("quark_d_write_mem_chunk error");
    }
    
    uint32_t flash_addr = (address & QUARK_D_MEM_SELECT_MASK);
    if ((address >> 28) == QUARK_D_INST_FLASH_FLAG) {
        for (uint32_t i = 0; i < count; i++) {
            uint32_t flash_page   = (flash_addr >> 12) & 0x7;
            uint32_t flash_offset = (flash_addr / 4) % 1024;
            if (!quark_d_inst_flash[flash_page].dirty) {
               quark_d_read_flash_shadow_copy(target, E_INST, flash_page);
               quark_d_inst_flash[flash_page].dirty = 1;
               //LOG_INFO("IF page marked as dirty. Addr: %08X Page: %d", address, flash_page);
            }
            quark_d_inst_flash[flash_page].mem[flash_offset] = data[i];
            flash_addr = flash_addr + 4;
        }
        return ERROR_OK;
    } else if ((address >> 28) == QUARK_D_DATA_FLASH_FLAG) {
        for (uint32_t i = 0; i < count; i++) {
            uint32_t flash_page   = (flash_addr >> 11) & 0x1;
            uint32_t flash_offset = (flash_addr / 4) % 512;
            if (!quark_d_data_flash[flash_page].dirty) {
                quark_d_read_flash_shadow_copy(target, E_DATA, flash_page);
                quark_d_data_flash[flash_page].dirty = 1;
                //LOG_INFO("DF page marked as dirty. Addr: %08X Page: %d", address, flash_page);
            }
            quark_d_data_flash[flash_page].mem[flash_offset] = data[i];
            flash_addr = flash_addr + 4;
        }
        return ERROR_OK;
    } else if ((address >> 28) == QUARK_D_INST_ROM_FLAG) {   
        LOG_INFO("Attempt write to ROM A: %08X C: %08X", address, count);
        return ERROR_OK;    
    } else {   
    }
    
    //original NC subroutine
    if ((retval = quark_d_debug_write_register(target, DEBUG_MEMORY_ACCESS_ADDRESS, address)) != ERROR_OK)
    {
        return retval;
    }
    if ((retval = quark_d_debug_write_register(target, DEBUG_MEMORY_ACCESS_COUNT, count & 0xFFFF)) != ERROR_OK)
    {
        return retval;
    }
    //start transaction
    if ((retval = quark_d_debug_write_register(target,
        DEBUG_MEMORY_ACCESS_CMD, DEBUG_MEMORY_ACCESS_CMD_START
        | DEBUG_MEMORY_ACCESS_CMD_SEQ | DEBUG_MEMORY_ACCESS_CMD_WRITE | DEBUG_MEMORY_ACCESS_CMD_ALL_BYTES)) != ERROR_OK)
    {
        return retval;
    }
    
    quark_d_jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    field.in_value = NULL;
    out_data[0] = DEBUG_MEMORY_ACCESS_WR_DATA;
    field.out_value = (void*)&out_data[0];
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    quark_d_jtag_set_instruction(target, TAP_STREAM);
    out_data[1] = TAP_START_CMD;
    field.num_bits = TAP_STREAM_LEN;
    for (uint32_t i = 0; i < count; ++i)
    {
        out_data[0] = data[i];
        field.out_value = (void*)out_data;
        jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    }
    //LOG_INFO("command buffer initialization is done");
    if (jtag_execute_queue() != ERROR_OK)
    {
        LOG_ERROR("mem chunk write failed");
        return ERROR_FAIL;
    }
    //LOG_INFO("WR: A %08X D %08X", address, data);
    return ERROR_OK;
}

int
quark_d_write_mem_word(struct target * target, uint32_t const address, uint32_t const data)
{
    static int first_run_flag = 1;
    if (first_run_flag)
    {
        LOG_INFO("Optimized memory load enabled");
        first_run_flag = 0;
    }
    
    //LOG_INFO("Write %08X %08X", address, data);
    uint32_t flash_addr = (address & QUARK_D_MEM_SELECT_MASK);
    if ((address >> 28) == QUARK_D_INST_FLASH_FLAG) {
        uint32_t flash_page   = (flash_addr >> 12) & 0x7;
        uint32_t flash_offset = (flash_addr / 4) % 1024;
        if (!quark_d_inst_flash[flash_page].dirty) {
           quark_d_read_flash_shadow_copy(target, E_INST, flash_page);
           quark_d_inst_flash[flash_page].dirty = 1;
           //LOG_INFO("IF page marked as dirty. Addr: %08X Page: %d", address, flash_page);
        }
        //LOG_INFO("Cache write %08X %08X", address, data);
        quark_d_inst_flash[flash_page].mem[flash_offset] = data;
        return ERROR_OK;
    } else if ((address >> 28) == QUARK_D_DATA_FLASH_FLAG) {
        uint32_t flash_page   = (flash_addr >> 11) & 0x1;
        uint32_t flash_offset = (flash_addr / 4) % 512;
        if (!quark_d_data_flash[flash_page].dirty) {
            quark_d_read_flash_shadow_copy(target, E_DATA, flash_page);
            quark_d_data_flash[flash_page].dirty = 1;
            //LOG_INFO("DF page marked as dirty. Addr: %08X Page: %d", address, flash_page);
        }
        //LOG_INFO("Cache write %08X %08X", address, data);
        quark_d_data_flash[flash_page].mem[flash_offset] = data;
        return ERROR_OK;
    } else if ((address >> 28) == QUARK_D_INST_ROM_FLAG) {   
        LOG_INFO("Attempt write to ROM A: %08X D: %08X", address, data);
        return ERROR_OK; 
    } else {   
    }
    
    //LOG_INFO("Normal write %08X %08X", address, data);
    struct scan_field field;
    static uint32_t out_data;

    quark_d_jtag_set_instruction(target, TAP_CMD);
    field.num_bits = TAP_CMD_LEN;
    out_data = TAP_CMD_WR;
    field.out_value = (void*)&out_data;
    field.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_ADDRESS;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = address;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    quark_d_jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    //-----------------------------------------------
    quark_d_jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_WR_DATA;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = data;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    //----------------------------------------------
    quark_d_jtag_set_instruction(target, TAP_ADDR);
    field.num_bits = TAP_ADDR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_WR);
    field.num_bits = TAP_WR_LEN;
    out_data = DEBUG_MEMORY_ACCESS_CMD_START | DEBUG_MEMORY_ACCESS_CMD_WRITE
        | DEBUG_MEMORY_ACCESS_CMD_ALL_BYTES;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_START);
    field.num_bits = TAP_START_LEN;
    out_data = TAP_START_CMD;
    field.out_value = (void*)&out_data;
    jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

    if (jtag_execute_queue() != ERROR_OK)
    {
        LOG_ERROR("register read failed");
        return ERROR_FAIL;
    }
    
    //LOG_INFO("WR: A %08X D %08X", address, data);
    return ERROR_OK;
}

int
quark_d_debug_write_register(struct target * target, uint32_t const address, uint32_t const data)
{
    struct scan_field field_cmd, field_addr, field_start, field_data;
    uint32_t cmd, addr, start, data_in;
    
    quark_d_jtag_set_instruction(target, TAP_CMD);
    field_cmd.num_bits = TAP_CMD_LEN;
    field_cmd.out_value = (void*)&cmd;
    cmd = TAP_CMD_WR;
    field_cmd.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field_cmd, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_ADDR);
    field_addr.num_bits = TAP_ADDR_LEN;
    field_addr.out_value = (void*)&addr;
    addr = address;
    field_addr.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field_addr, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_WR);
    field_data.num_bits = TAP_WR_LEN;
    data_in = data;
    field_data.out_value = (void*)&data_in;
    field_data.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field_data, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_START);
    field_start.num_bits = TAP_START_LEN;
    field_start.out_value = (void*)&start;
    start = TAP_START_CMD;
    field_start.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field_start, TAP_IDLE);

    if (!inside_macro_transaction)
    {
        if (jtag_execute_queue() != ERROR_OK)
        {
            LOG_ERROR("register read failed");
            return ERROR_FAIL;
        }
    }

    return ERROR_OK;
}

int
quark_d_debug_read_register(struct target * target, uint32_t const address, uint32_t *const data)
{
    struct scan_field field_cmd, field_addr, field_start, field_data;
    static uint32_t cmd;
    static uint32_t addr;
    static uint32_t start;
    
    quark_d_jtag_set_instruction(target, TAP_CMD);
    field_cmd.num_bits = TAP_CMD_LEN;
    field_cmd.out_value = (void*)&cmd;
    cmd = TAP_CMD_RD;
    field_cmd.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field_cmd, TAP_IDLE);
  
    quark_d_jtag_set_instruction(target, TAP_ADDR);
    field_addr.num_bits = TAP_ADDR_LEN;
    field_addr.out_value = (void*)&addr;
    addr = address;
    field_addr.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field_addr, TAP_IDLE);
    
    quark_d_jtag_set_instruction(target, TAP_START);
    field_start.num_bits = TAP_START_LEN;
    field_start.out_value =(void*)&start;
    start = 0x1;
    field_start.in_value = NULL;
    jtag_add_dr_scan(target->tap, 1, &field_start, TAP_IDLE);
    
   
    quark_d_jtag_set_instruction(target, TAP_RD);
    field_data.num_bits = TAP_RD_LEN;
    field_data.out_value = NULL;
    *data = 0;
    field_data.in_value = (void*)data;
    jtag_add_dr_scan(target->tap, 1, &field_data, TAP_IDLE);
    
    if (!inside_macro_transaction)
    {
        if (jtag_execute_queue() != ERROR_OK)
        {
            LOG_ERROR("register read failed");
            return ERROR_FAIL;
        }
    }

    return ERROR_OK;
}

///update cache 
int
quark_d_save_context(struct target * target)
{
    uint32_t nc_state;
    struct quark_d_arch * const arch_info = (struct quark_d_arch *)target->arch_info;
    macro_transaction_begin();
        quark_d_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state);

        for (unsigned i = 0; i < 10; ++i) //NC has only 10 regs
        {
            quark_d_debug_read_register(target, DEBUG_REGISTERS_ACCESS_BASE + i * 4, (uint32_t*)(arch_info->core_cache->reg_list[i].value));
        }
    macro_transaction_end();
    
    //issue error if we are still running
    if (!(nc_state & DEBUG_CONTROL_STATUS_HALTED))
    {
        LOG_ERROR("quark_d1x00 is not halted quark_d_save_context");
    }
    for (unsigned i = 0; i < 10; ++i)
    {
        arch_info->core_cache->reg_list[i].dirty = 0;
        arch_info->core_cache->reg_list[i].valid = 1;
    }
    return ERROR_OK;
}

///update register values in HW from cache
int
quark_d_restore_context(struct target*  target)
{
    struct quark_d_arch *arch_info = (struct quark_d_arch *)target->arch_info;
    uint32_t reg_value;
    uint32_t nc_state;

    macro_transaction_begin();
        quark_d_debug_read_register(target, DEBUG_CONTROL_STATUS, &nc_state);

        for (int i = 0; i < 10; ++i) //NC has only 10 regs
        {
            reg_value = *(uint32_t*)(arch_info->core_cache->reg_list[i].value);
            quark_d_debug_write_register(target, DEBUG_REGISTERS_ACCESS_BASE + i * 4, reg_value);
        }
    macro_transaction_end();

    if (!(nc_state & DEBUG_CONTROL_STATUS_HALTED))
    {
        LOG_ERROR("quark_d10xx is not halted quark_d_restore_context");
    }
    for (unsigned i = 0; i < 10; ++i)
    {
        arch_info->core_cache->reg_list[i].dirty = 0;
        arch_info->core_cache->reg_list[i].valid = 1;
    }
    return ERROR_OK;
}

int
quark_d_read_flash_shadow_copy(struct target* target, quark_d_flash_type flash_type, uint32_t page_number)
{
    uint32_t start_addr;
    uint32_t count;
    if (flash_type == E_INST) {
        if (page_number >= 8) {
            LOG_ERROR("Instruction page_number %d in quark_d_read_flash_shadow_copy is out of bounds!", page_number);
            return ERROR_FAIL;
        }
        start_addr  = QUARK_D_INST_FLASH_BASE + 4096 * page_number;
        count       = 1024;
    } else if (flash_type == E_DATA) {
        if (page_number >= 2) {
            LOG_ERROR("Data page_number %d in quark_d_read_flash_shadow_copy is out of bounds!", page_number);
            return ERROR_FAIL;
        }
        start_addr  = QUARK_D_DATA_FLASH_BASE + 2048 * page_number;
        count       = 512;
    } else {
        LOG_ERROR("Internal error in quark_d_read_flash_shadow_copy!");
        return ERROR_FAIL;
    }

    for (uint32_t i = 0; i < count; i++) {
        uint32_t data;
        if (quark_d_read_mem_word(target, start_addr, &data) != ERROR_OK) {
            LOG_ERROR("Read error in quark_d_read_flash_shadow_copy!");
            return ERROR_FAIL;
        }
        
        if (flash_type == E_INST) {
            quark_d_inst_flash[page_number].mem[i] = data;
        } else {
            quark_d_data_flash[page_number].mem[i] = data;
        }

        start_addr = start_addr + 4;
    }

    if (flash_type == E_INST) {
    	memcpy(quark_d_inst_flash[page_number].mem_o, quark_d_inst_flash[page_number].mem, 1024 * 4);
    } else {
    	memcpy(quark_d_data_flash[page_number].mem_o, quark_d_data_flash[page_number].mem, 512 * 4);
    }
    return ERROR_OK;
}

static void
quark_d_delay(struct target* t, unsigned c)
{
    jtag_add_clocks(c*50);
}

int 
quark_d_df_erase(struct target* target, uint32_t page_number) {
    uint32_t tmp;

    unsigned poll_count = 0;
    const unsigned poll_count_limit = 1000;

    macro_transaction_begin();
        quark_d_write_mem_word(target, QUARK_D_FLASH_DATA_WR_ADDR_REG, QUARK_D_DATA_FLASH_BASE + 2048 * page_number);
        quark_d_write_mem_word(target, QUARK_D_FLASH_ERASE_WRITE_CNTL_REG, 0x009D0008);
        quark_d_delay(target, QUARK_D_FLASH_ERASE_TCK_TIME / jtag_get_speed_khz());
        quark_d_read_mem_word(target, QUARK_D_FLASH_STATUS_REG, &tmp);
    macro_transaction_end();

    while (!(tmp & 0x02)) {
        macro_transaction_begin();
            quark_d_read_mem_word(target, QUARK_D_FLASH_STATUS_REG, &tmp);
        macro_transaction_end();

        poll_count++;
        if (poll_count > poll_count_limit) {
            LOG_ERROR("DF flash erase error, status: %08X, page %d", tmp, page_number);
            return ERROR_FAIL;
	}

    }
    
    macro_transaction_begin();
        quark_d_write_mem_word(target, QUARK_D_FLASH_STATUS_REG, 0x06);
    macro_transaction_end(); 
    
    return ERROR_OK;
}

static int 
quark_d_df_prog(struct target* target, uint32_t address, uint32_t data) {
    uint32_t tmp;

    unsigned poll_count = 0;
    const unsigned poll_count_limit = 100;
    
    macro_transaction_begin();
        quark_d_write_mem_word(target, QUARK_D_FLASH_DATA_WR_ADDR_REG, address);
        quark_d_write_mem_word(target, QUARK_D_FLASH_DATA_WR_DATA_REG, data);
        quark_d_write_mem_word(target, QUARK_D_FLASH_ERASE_WRITE_CNTL_REG, 0x009D0002);
        quark_d_delay(target, QUARK_D_FLASH_PROG_TCK_TIME / jtag_get_speed_khz());
        quark_d_read_mem_word(target, QUARK_D_FLASH_STATUS_REG, &tmp);
    macro_transaction_end();

    while (!(tmp & 0x04)) {
        macro_transaction_begin();
            quark_d_read_mem_word(target, QUARK_D_FLASH_STATUS_REG, &tmp);
        macro_transaction_end();

        poll_count++;
        if (poll_count > poll_count_limit) {
            LOG_ERROR("Data flash prog error %08X address is %08X, data is %08X", tmp, address, data);
            return ERROR_FAIL;
	}
    }

    macro_transaction_begin();
        quark_d_write_mem_word(target, QUARK_D_FLASH_STATUS_REG, 0x06);
    macro_transaction_end();
    
    return ERROR_OK;
}

static int 
quark_d_cf_erase(struct target* target, uint32_t page_number) {
    uint32_t data;

    unsigned poll_count = 0;
    const unsigned poll_count_limit = 1000;
    
    macro_transaction_begin();
        quark_d_write_mem_word(target, QUARK_D_FLASH_INSTR_WR_ADDR_REG, QUARK_D_INST_FLASH_BASE + 4096 * page_number);
        quark_d_write_mem_word(target, QUARK_D_FLASH_ERASE_WRITE_CNTL_REG, 0x009D0004);
        quark_d_delay(target, QUARK_D_FLASH_ERASE_TCK_TIME / jtag_get_speed_khz() + 1000);
        quark_d_read_mem_word(target, QUARK_D_FLASH_STATUS_REG, &data);
    macro_transaction_end(); 

    while (!(data & 0x02)) {
        macro_transaction_begin();
            quark_d_read_mem_word(target, QUARK_D_FLASH_STATUS_REG, &data);
        macro_transaction_end();

        poll_count++;
        if (poll_count > poll_count_limit) {
            LOG_ERROR("CF flash erase error, status: %08X, page %d", data, page_number);
            return ERROR_FAIL;
	}

    }
    
    macro_transaction_begin();
        quark_d_write_mem_word(target, QUARK_D_FLASH_STATUS_REG, 0x06);
    macro_transaction_end(); 
    
    return ERROR_OK;
}

void
quark_d_debug_dump(struct target* t)
{
    uint32_t val = 0, i;
    LOG_ERROR("---------------");
    // Stop core
    quark_d_debug_write_register(t, DEBUG_CONTROL_COMMAND, DEBUG_CONTROL_COMMAND_HALT);
    // quark_d_debug_read_register(t, DEBUG_CONTROL_STATUS, &val);
    LOG_ERROR("DEBUG_CONTROL_STATUS %X", val);


    for (i = 0; i < 10; ++i)
    {
        quark_d_debug_read_register(t, DEBUG_REGISTERS_ACCESS_BASE + i * 4, &val);
        LOG_ERROR("%X %X",i * 4 + DEBUG_REGISTERS_ACCESS_BASE, val);
    }
}

int 
quark_d_cf_prog(struct target* target, uint32_t address, uint32_t data0, uint32_t data1) {
    uint32_t tmp;

    unsigned poll_count = 0;
    const unsigned poll_count_limit = 100;

    macro_transaction_begin();
        quark_d_delay(target, 10);
        quark_d_write_mem_word(target, QUARK_D_FLASH_INSTR_WR_ADDR_REG, address);
        quark_d_write_mem_word(target, QUARK_D_FLASH_INSTR_WR_DATA_0_REG, data0);
        quark_d_write_mem_word(target, QUARK_D_FLASH_INSTR_WR_DATA_1_REG, data1);
        quark_d_write_mem_word(target, QUARK_D_FLASH_ERASE_WRITE_CNTL_REG, 0x009D0001);
        quark_d_delay(target, QUARK_D_FLASH_PROG_TCK_TIME / jtag_get_speed_khz());
        quark_d_read_mem_word(target, QUARK_D_FLASH_STATUS_REG, &tmp);
    macro_transaction_end(); 
    
    while (!(tmp & 0x04)) {
        macro_transaction_begin();
            quark_d_read_mem_word(target, QUARK_D_FLASH_STATUS_REG, &tmp);
        macro_transaction_end();

        poll_count++;
        if (poll_count > poll_count_limit) {
            LOG_ERROR("Inst flash prog error %08X address is %08X, data is %08X, %08X",
                tmp, address, data0, data1);
            return ERROR_FAIL;
	}
    }
    macro_transaction_begin();
        quark_d_write_mem_word(target, QUARK_D_FLASH_STATUS_REG, 0x06);
    macro_transaction_end();
    return ERROR_OK;
}

int
quark_d_write_flash_shadow_copy(struct target* target) {
    uint32_t start_addr;
    uint32_t count;
    int ret_val;

    //check data flash shadow copy
    for (uint32_t i = 0; i < 2; i++) {
	if (memcmp(quark_d_data_flash[i].mem, quark_d_data_flash[i].mem_o, 512 * 4) == 0) continue;
        if (quark_d_data_flash[i].dirty) {
            //LOG_INFO("Erasing DF page %d", i);
            start_addr  = QUARK_D_DATA_FLASH_BASE + 2048 * i;
            count       = 512;
           
            if ((ret_val = quark_d_df_erase(target, i)) != ERROR_OK) {
                return ret_val;
            }
            
            for (uint32_t j = 0; j < count; j++) {
                if (quark_d_data_flash[i].mem[j] != 0xFFFFFFFF) {
                    //LOG_INFO("i %d j %d sa %08X", i, j, start_addr);
                    if ((ret_val = quark_d_df_prog(target, start_addr, quark_d_data_flash[i].mem[j])) != ERROR_OK) {
                        return ret_val;
                    }
                }
                start_addr = start_addr + 4;
            }

            quark_d_data_flash[i].dirty = 0;
        }
    }
    
    //check inst flash shadow copy
    for (uint32_t i = 0; i < 8; i++) {
	if (memcmp(quark_d_inst_flash[i].mem, quark_d_inst_flash[i].mem_o, 1024 * 4) == 0) continue;
        if (quark_d_inst_flash[i].dirty) {
            //LOG_INFO("Erasing IF page %d", i);
            start_addr  = QUARK_D_INST_FLASH_BASE + 4096 * i;
            count       = 1024;
            
            if ((ret_val = quark_d_cf_erase(target, i)) != ERROR_OK) {
                return ret_val;
            }
            
            for (uint32_t j = 0; j < count; j = j + 4) {
                if ((quark_d_inst_flash[i].mem[j] != 0xFFFFFFFF)   ||
                    (quark_d_inst_flash[i].mem[j+1] != 0xFFFFFFFF) ||
                    (quark_d_inst_flash[i].mem[j+2] != 0xFFFFFFFF) ||
                    (quark_d_inst_flash[i].mem[j+3] != 0xFFFFFFFF)) {
                    
                    if ((ret_val = quark_d_cf_prog(target, start_addr, quark_d_inst_flash[i].mem[j], quark_d_inst_flash[i].mem[j+2])) != ERROR_OK) {
                        return ret_val;
                    }
                    
                    if ((ret_val = quark_d_cf_prog(target, start_addr + 4, quark_d_inst_flash[i].mem[j+1], quark_d_inst_flash[i].mem[j+3])) != ERROR_OK) {
                        return ret_val;
                    }
                }
                
                start_addr = start_addr + 16;
            }
            
            quark_d_inst_flash[i].dirty = 0;
        }
    }    
    
    return ERROR_OK;
}

