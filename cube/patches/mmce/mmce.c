/***************************************************************************
# MMCE Read code for GC/Wii via SD on EXI
# bbsan2k 2025
#**************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include "common.h"
#include "dolphin/exi.h"
#include "dolphin/os.h"
#include "emulator.h"
#include "frag.h"
#include "interrupt.h"

typedef enum {
    MMCE_IDLE,
    MMCE_WAIT_TC
} mmce_state_e;

#ifndef MMCE_DEBUG
#define MMCE_DEBUG 0
#endif

#if MMCE_DEBUG
#define DEBUG(string) WriteUARTN(string, sizeof(string))
#else
#define DEBUG(string) do { } while (0)
#endif

#ifndef QUEUE_SIZE
#define QUEUE_SIZE                       2
#endif

#define SECTOR_SIZE                      512U

#ifndef MMCE_START_READY_DELAY_LOOPS
#define MMCE_START_READY_DELAY_LOOPS     20U
#endif

#ifndef MMCE_CONTINUE_READY_DELAY_LOOPS
#define MMCE_CONTINUE_READY_DELAY_LOOPS  10U
#endif

#ifndef MMCE_DREADY_TIMEOUT_LOOPS
#define MMCE_DREADY_TIMEOUT_LOOPS        1000000U
#endif

#ifndef MMCE_READY_DELAY_RETRY_STEP_LOOPS
#define MMCE_READY_DELAY_RETRY_STEP_LOOPS 2U
#endif

#ifndef WRITE
#define WRITE write
#endif

#define exi_cpr                          (*(u8*)VAR_EXI_CPR)
#define exi_channel                      (*(u8*)VAR_EXI_SLOT & 0x3)
#define exi_device                       ((*(u8*)VAR_EXI_SLOT & 0xC) >> 2)
#define exi_regs                         (*(vu32**)VAR_EXI_REGS)

typedef struct mmce_Tag {
    frag_callback done_callback;
    void *buffer;
    void *user_buffer;
    uint32_t length;
    uint32_t offset;
    uint32_t sector;
    uint32_t count;
    uint32_t current_block;
    bool write;
} mmce_t;

static uint8_t *mmce_buffer = (uint8_t *)&VAR_SECTOR_BUF;
static mmce_state_e mmce_state = MMCE_IDLE;
static mmce_t mmce_queue[QUEUE_SIZE];
static mmce_t *mmce_op;
static uint32_t current_start_ready_delay = MMCE_START_READY_DELAY_LOOPS;
static uint32_t current_continue_ready_delay = MMCE_CONTINUE_READY_DELAY_LOOPS;

static void mmce_start_read(s32 chan, OSContext *context);
static void mmce_read(s32 chan, OSContext *context);
static void mmce_handle_tc_interrupt(OSInterrupt interrupt);
static void mmce_complete_operation(void);
static void mmce_delay_loops(uint32_t loops);
static bool mmce_check_interrupt(void);
static uint32_t mmce_ready_delay_loops(bool first_block);
static void mmce_increase_ready_delay(bool first_block);
static void mmce_retry_current_operation(bool first_block);
static void mmce_issue_read_command(uint32_t sector, uint32_t count);
static bool wait_dready(bool first_block);
static void exi_select(void);
static void exi_deselect(void);
static void exi_imm_write(u32 data, int len, bool sync);
static void exi_dma_read(void *data, int len, bool sync);

static void mmce_interrupt_handler(OSInterrupt interrupt, OSContext *context)
{
    (void)context;

    if ((OS_INTERRUPT_EXI_0_TC + (3 * exi_channel) == interrupt)
        && (MMCE_WAIT_TC == mmce_state)) {
        mmce_handle_tc_interrupt(interrupt);
    }
}

static void mmce_delay_loops(uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops; i++) {
        asm volatile("nop");
    }
}

static bool mmce_check_interrupt(void)
{
    return !!(exi_regs[0] & 0x2);
}

static uint32_t mmce_ready_delay_loops(bool first_block)
{
    return first_block ? current_start_ready_delay
                       : current_continue_ready_delay;
}

static void mmce_increase_ready_delay(bool first_block)
{
    uint32_t *delay = first_block ? &current_start_ready_delay
                                  : &current_continue_ready_delay;

    if (*delay <= UINT32_MAX - MMCE_READY_DELAY_RETRY_STEP_LOOPS) {
        *delay += MMCE_READY_DELAY_RETRY_STEP_LOOPS;
    }
}

static void mmce_retry_current_operation(bool first_block)
{
    if (mmce_op == NULL) {
        return;
    }

    mmce_state = MMCE_IDLE;
    mmce_increase_ready_delay(first_block);
    //EXIUnlock(exi_channel);
    mmce_op->length = 0U;
    mmce_complete_operation();
}

static void exi_select(void)
{
    exi_regs[0] = (exi_regs[0] & 0x405) | ((exi_cpr << 4) & 0x3F0);
}

static void exi_deselect(void)
{
    exi_regs[0] &= 0x405;
}

static void exi_imm_write(u32 data, int len, bool sync)
{
    exi_regs[4] = data;
    exi_regs[3] = ((len - 1) << 4) | (EXI_WRITE << 2) | 1;
    while (sync && (exi_regs[3] & 1));
}

static void exi_dma_read(void *data, int len, bool sync)
{
    exi_regs[1] = (unsigned long)data;
    exi_regs[2] = len;
    exi_regs[3] = 3;

    while (sync && (exi_regs[3] & 1));
}

static void mmce_issue_read_command(uint32_t sector, uint32_t count)
{
    exi_regs[0] |= 0x2;
    exi_select();
    exi_imm_write(0x8B << 24 | 0x20 << 16 | ((sector >> 24) & 0xFF) << 8 | ((sector >> 16) & 0xFF), 4, true);
    exi_imm_write(((sector >> 8) & 0xFF) << 24 | (sector & 0xFF) << 16 | ((count >> 8) & 0xFF) << 8 | (count & 0xFF), 4, true);
    exi_deselect();
}

static bool wait_dready(bool first_block)
{
    uint32_t loops = MMCE_DREADY_TIMEOUT_LOOPS;

    if (exi_channel >= EXI_CHANNEL_MAX) {
        __builtin_trap();
    }
    if (mmce_op == NULL) {
        return false;
    }

    mmce_delay_loops(mmce_ready_delay_loops(first_block)); // This is required as for certain reasons EXI does not settle

    while (!mmce_check_interrupt() && loops > 0) {
        asm volatile("nop");
        loops--;
    }

    if (!mmce_check_interrupt()) {
        exi_regs[0] |= 0x2;
        DEBUG("MMCE: DREADY wait timed out\n");
        return false;
    }

    exi_regs[0] |= 0x2;
    return true;
}

static void mmce_read(s32 chan, OSContext *context)
{
    if (mmce_op == NULL) {
        return;
    }
    if (!EXILock(exi_channel, exi_device, mmce_read)) {
        return;
    }

    OSInterrupt interrupt = OS_INTERRUPT_EXI_0_TC + (3 * exi_channel);
    void *read_buffer = mmce_op->user_buffer;

    mmce_state = MMCE_WAIT_TC;
    exi_regs[0] |= 0x8;
    DCInvalidateRange(__builtin_assume_aligned(mmce_buffer, 32), SECTOR_SIZE);
    if (mmce_op->current_block == 0 && mmce_op->offset != 0) {
        read_buffer = mmce_buffer;
    } else if ((mmce_op->current_block == mmce_op->count - 1)
               && ((mmce_op->length + mmce_op->offset) % SECTOR_SIZE) != 0) {
        read_buffer = mmce_buffer;
    } else if (((uintptr_t)mmce_op->user_buffer % 32U) != 0U) {
        read_buffer = mmce_buffer;
    }

    exi_select();
    exi_imm_write(0x8B << 24 | 0x21 << 16 | 0x00 << 8 | 0xFF, 3, true);
    set_interrupt_handler(interrupt, mmce_interrupt_handler);
    unmask_interrupts(OS_INTERRUPTMASK(interrupt)
                    & (OS_INTERRUPTMASK_EXI_0_TC | OS_INTERRUPTMASK_EXI_1_TC | OS_INTERRUPTMASK_EXI_2_TC));
    exi_dma_read(read_buffer, SECTOR_SIZE, false);
}

static void mmce_handle_tc_interrupt(OSInterrupt interrupt)
{
    if ((exi_regs[3] & 1) != 0) {
        return;
    }

    mask_interrupts(OS_INTERRUPTMASK(interrupt)
                  & (OS_INTERRUPTMASK_EXI_0_TC | OS_INTERRUPTMASK_EXI_1_TC | OS_INTERRUPTMASK_EXI_2_TC));
    exi_deselect();
    EXIUnlock(exi_channel);

    if (mmce_op == NULL) {
        return;
    }

    {
        uint32_t partial_len = SECTOR_SIZE;
        uint32_t off = 0;

        if (mmce_op->current_block == 0 && mmce_op->offset != 0) {
            partial_len = MIN(mmce_op->length, SECTOR_SIZE - mmce_op->offset);
            off = mmce_op->offset;
            memcpy(mmce_op->user_buffer, mmce_buffer + off, partial_len);
        } else if (mmce_op->current_block == mmce_op->count - 1
                   && (mmce_op->length + mmce_op->offset) % SECTOR_SIZE != 0) {
            partial_len = (mmce_op->length + mmce_op->offset) % SECTOR_SIZE;
            memcpy(mmce_op->user_buffer, mmce_buffer + off, partial_len);
        } else if (((uintptr_t)mmce_op->user_buffer % 32U) != 0U) {
            memcpy(mmce_op->user_buffer, mmce_buffer, SECTOR_SIZE);
        }

        mmce_op->user_buffer = (uint8_t *)mmce_op->user_buffer + partial_len;
        mmce_op->current_block++;
    }

    if (mmce_op->current_block < mmce_op->count) {
        if (wait_dready(false)) {
            mmce_read(exi_channel, NULL);
        } else {
            DEBUG("MMCE: DREADY wait timed out during multi-block read\n");
            mmce_retry_current_operation(false);
        }
        return;
    }

    mmce_state = MMCE_IDLE;
    EXIUnlock(exi_channel);
    mmce_complete_operation();
}

static void mmce_complete_operation(void)
{
    if (mmce_op == NULL) {
        return;
    }

    if (mmce_op->done_callback) {
        mmce_op->done_callback(mmce_op->buffer, mmce_op->length);
    }
    memset(mmce_op, 0, sizeof(mmce_t));

    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (mmce_queue[i].done_callback) {
            mmce_op = &mmce_queue[i];
            mmce_start_read(exi_channel, NULL);
            return;
        }
    }
    mmce_op = NULL;
}

static void mmce_start_read(s32 chan, OSContext *context)
{
    (void)chan;
    (void)context;

    if (exi_channel >= EXI_CHANNEL_MAX) {
        __builtin_trap();
    }

    if (mmce_op == NULL) {
        return;
    }
    if (!EXILock(exi_channel, exi_device, mmce_start_read)) {
        return;
    }

    mmce_issue_read_command(mmce_op->sector, mmce_op->count);
    EXIUnlock(exi_channel);

    if (wait_dready(true)) {
        mmce_read(exi_channel, NULL);
    } else {
        DEBUG("MMCE: DREADY wait timed out at start of read\n");
        mmce_retry_current_operation(true);
    }
}

bool do_read_write_async(void *buffer, uint32_t length, uint32_t offset, uint64_t sector, bool write, frag_callback callback)
{
    mmce_t op;

    if (write) {
        return false;
    }

    op.sector = (uint32_t)sector + (offset / SECTOR_SIZE);
    op.offset = offset % SECTOR_SIZE;
    op.count = (length + SECTOR_SIZE - 1 + op.offset) / SECTOR_SIZE;
    op.length = length;
    op.buffer = buffer;
    op.user_buffer = buffer;
    op.done_callback = callback;
    op.write = write;
    op.current_block = 0;

    DEBUG("MMCE async read queued\n");

    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (!mmce_queue[i].done_callback) {
            mmce_queue[i] = op;
            if (mmce_op == NULL) {
                mmce_op = &mmce_queue[i];
                mmce_start_read(exi_channel, NULL);
            }
            return true;
        }
    }
    return false;
}