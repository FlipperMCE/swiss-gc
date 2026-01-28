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
    MMCE_WAIT_EXI,
    MMCE_WAIT_TC,
    MMCE_UNKNOWN
} mmce_state_e;

#define DEBUG(string) WriteUARTN(string, sizeof(string))
char text[16] = {0};

#ifndef QUEUE_SIZE
#define QUEUE_SIZE          2
#endif
#define SECTOR_SIZE         512
#ifndef WRITE
#define WRITE               write
#endif

#define exi_cpr             (*(u8*)VAR_EXI_CPR)
#define exi_channel         (*(u8*)VAR_EXI_SLOT & 0x3)
#define exi_device          ((*(u8*)VAR_EXI_SLOT & 0xC) >> 2)
#define exi_regs            (*(vu32**)VAR_EXI_REGS)

bool mmce_in_progress = false;

static uint8_t* mmce_buffer =  (uint8_t*)&VAR_SECTOR_BUF;

static mmce_state_e mmce_state = MMCE_IDLE;

typedef struct mmce_Tag {
    frag_callback done_callback;
    void* buffer;
    void* user_buffer;
    uint32_t length;
    uint32_t offset;
    uint32_t sector;
    uint32_t count;
    uint32_t current_block;
    bool write;
} mmce_t;

static mmce_t mmce_queue[QUEUE_SIZE];
static mmce_t *mmce_op;

static void wait_dready(s32 chan, OSContext *context);
static void mmce_start_read(s32 chan, OSContext *context);
static void mmce_read(void);
static void mmce_handle_exi_interrupt(OSInterrupt interrupt);
static void mmce_handle_tc_interrupt(OSInterrupt interrupt);
static void mmce_complete_operation(void);

void intToStr(int N, char *str) {
    int i = 0;
  
    // Save the copy of the number for sign
    int sign = N;

    // If the number is negative, make it positive
    if (N < 0)
        N = -N;

    // Extract digits from the number and add them to the
    // string
    while (N > 0) {
      
        // Convert integer digit to character and store
      	// it in the str
        str[i++] = N % 10 + '0';
      	N /= 10;
    } 

    // If the number was negative, add a minus sign to the
    // string
    if (sign < 0) {
        str[i++] = '-';
    }

    // Null-terminate the string
    str[i] = '\0';

    // Reverse the string to get the correct order
    for (int j = 0, k = i - 1; j < k; j++, k--) {
        char temp = str[j];
        str[j] = str[k];
        str[k] = temp;
    }
}

void u8toHex(uint8_t val, char* str)
{
    const char hexChars[] = "0123456789ABCDEF";
    str[0] = hexChars[(val >> 4) & 0x0F];
    str[1] = hexChars[val & 0x0F];
    str[2] = '\0';
}

static void printBufferHex(uint8_t* buffer, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        char hex[3];
        u8toHex(buffer[i], hex);
        DEBUG(hex);
        DEBUG(" ");
    }
    DEBUG("\n");
}

static bool exi_selected()
{
    return !!(exi_regs[0] & 0x380);
}

static void exi_select()
{
    exi_regs[0] = (exi_regs[0] & 0x405) | ((exi_cpr << 4) & 0x3F0);
}


static void exi_deselect()
{
    exi_regs[0] &= 0x405;
}

static void mmce_interrupt_handler(OSInterrupt interrupt, OSContext *context)
{
    if (((OS_INTERRUPT_EXI_0_EXI + (3 * exi_channel)) == interrupt) 
        && (MMCE_WAIT_EXI == mmce_state)) {
        mmce_handle_exi_interrupt(interrupt);
    } else if ((OS_INTERRUPT_EXI_0_TC + (3 * exi_channel) == interrupt) 
                && (MMCE_WAIT_TC == mmce_state)) {
        mmce_handle_tc_interrupt(interrupt);
    }
}

static void mmce_handle_exi_interrupt(OSInterrupt interrupt)
{
    // This is EXI interrupt, signalling DREADY
    exi_regs[0] |= 0x2; // Reset IRQ
    //DEBUG("EXI_INT\n");
    mask_interrupts(OS_INTERRUPTMASK(interrupt) & (OS_INTERRUPTMASK_EXI_0_EXI | OS_INTERRUPTMASK_EXI_1_EXI | OS_INTERRUPTMASK_EXI_2_EXI));

    // Start reading/writing
    mmce_read();
}

static void mmce_handle_tc_interrupt(OSInterrupt interrupt)
{
    // This is TC_Interrupt - transfer complete
    if ((exi_regs[3] & 1))
        return; // Transfer not yet finished

    mask_interrupts(OS_INTERRUPTMASK(interrupt) & (OS_INTERRUPTMASK_EXI_0_TC | OS_INTERRUPTMASK_EXI_1_TC | OS_INTERRUPTMASK_EXI_2_TC));
    exi_deselect();
    
    // Handle the completed block
    if (mmce_op) {
        uint32_t partial_len = SECTOR_SIZE;
        uint32_t off = 0;
        if (mmce_op->current_block == 0 && mmce_op->offset != 0) {
            // First block with offset, copy from mmce_buffer to user buffer
            partial_len = MIN(mmce_op->length, SECTOR_SIZE - mmce_op->offset);
            off = mmce_op->offset;
            memcpy(mmce_op->user_buffer, mmce_buffer + off, partial_len);

        } else if (mmce_op->current_block == mmce_op->count - 1 && (mmce_op->length + mmce_op->offset) % SECTOR_SIZE != 0) {
            // Last block with partial data, copy to user buffer
            partial_len = (mmce_op->length + mmce_op->offset) % SECTOR_SIZE;
            memcpy(mmce_op->user_buffer, mmce_buffer + off, partial_len);
        } else if ((uint32_t)mmce_op->user_buffer % 32 != 0) {
            // Unaligned user buffer, copy via mmce_buffer
            off = 0;
            memcpy(mmce_op->user_buffer, mmce_buffer, SECTOR_SIZE);
        }

        //printBufferHex(mmce_op->user_buffer, 20);
        mmce_op->user_buffer += partial_len;

        mmce_op->current_block++;

        if (mmce_op->current_block < mmce_op->count) {
            // More blocks to read, send next read block
            mmce_state = MMCE_WAIT_EXI;

            OSInterrupt exi_interrupt = OS_INTERRUPT_EXI_0_EXI + (3 * exi_channel);
            
            unmask_interrupts(OS_INTERRUPTMASK(exi_interrupt) & 
                            (OS_INTERRUPTMASK_EXI_0_EXI | OS_INTERRUPTMASK_EXI_1_EXI | OS_INTERRUPTMASK_EXI_2_EXI));
            //wait_dready(exi_channel, NULL);
            //mmce_read();
        } else {
            // All blocks read, complete operation
            mmce_state = MMCE_IDLE;
            EXIUnlock(exi_channel);
            mmce_complete_operation();
        }
    }
}

static void mmce_complete_operation(void)
{
    if (!mmce_op) return;
    
    // Clear the operation
    // Call the callback
    if (mmce_op->done_callback) {
        mmce_op->done_callback(mmce_op->buffer, mmce_op->length);
    }
    memset(mmce_op, 0, sizeof(mmce_t));
    
    // Start next queued operation
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (mmce_queue[i].done_callback) {
            mmce_op = &mmce_queue[i];
            mmce_start_read(exi_channel, NULL);
            return;
        }
    }
    mmce_in_progress = false;
    mmce_op = NULL;
}

static void exi_imm_write(u32 data, int len, bool sync)
{
    exi_regs[4] = data;
    // Tell EXI if this is a read or a write
    exi_regs[3] = ((len - 1) << 4) | (EXI_WRITE << 2) | 1;
    // Wait for it to do its thing
    while (sync && (exi_regs[3] & 1));
}

static u32 exi_imm_read(int len, bool sync)
{
    exi_regs[4] = ~0;
    // Tell EXI if this is a read or a write
    exi_regs[3] = ((len - 1) << 4) | (EXI_READ << 2) | 1;

    if (sync) {
        // Wait for it to do its thing
        while (exi_regs[3] & 1);
        // Read the 4 byte data off the EXI bus
        return exi_regs[4] >> ((4 - len) * 8);
    }
    return 0;
}

static void exi_dma_write(void* data, int len, bool sync)
{
    exi_regs[1] = (unsigned long)data;
    exi_regs[2] = len;
    exi_regs[3] = (EXI_WRITE << 2) | 3;
    while (sync && (exi_regs[3] & 1));
}

static void exi_dma_read(void* data, int len, bool sync)
{
    exi_regs[1] = (unsigned long)data;
    exi_regs[2] = len;
    exi_regs[3] = 3;

    while (sync && (exi_regs[3] & 1));
}

static void wait_dready(s32 chan, OSContext *context) {	
    if (exi_channel >= EXI_CHANNEL_MAX)
        __builtin_trap();

    bool ret = false;
    
    while (!ret) {
        DEBUG(".");
        exi_select();
        exi_imm_write(0x8B << 24 | 0x24 << 16 | 0x00 << 8 | 0xFF, 3, true);
        ret = (((exi_imm_read(1, true)) & 0xFF) != 0x00);
        exi_deselect();
    }
}

static void mmce_read(void) {
    if (mmce_op) {
        //DEBUG("MMCE Read\n");

        OSInterrupt interrupt = OS_INTERRUPT_EXI_0_TC + (3 * exi_channel);
        mmce_state = MMCE_WAIT_TC;
        exi_regs[0] |= 0x8; // Reset IRQ
        void *read_buffer = mmce_op->user_buffer; // = (mmce_op->current_block == 0 && mmce_op->offset != 0) ? mmce_buffer : (mmce_op->user_buffer + (mmce_op->current_block * SECTOR_SIZE - (mmce_op->current_block > 0 ? mmce_op->offset : 0)));
        DCInvalidateRange(__builtin_assume_aligned(mmce_buffer, 32), SECTOR_SIZE);
        if (mmce_op->current_block == 0 && mmce_op->offset != 0) {
            read_buffer = mmce_buffer;
        } else if ((mmce_op->current_block == mmce_op->count - 1) 
                && ((mmce_op->length + mmce_op->offset) % SECTOR_SIZE) != 0) {
            read_buffer = mmce_buffer;
        } else if ((uint32_t)mmce_op->user_buffer % 32 != 0) {
            read_buffer = mmce_buffer;// + (mmce_op->current_block * SECTOR_SIZE - (mmce_op->current_block > 0 ? mmce_op->offset : 0));
        }
//        read_buffer = mmce_buffer;
    
        exi_select();
        
        exi_imm_write(0x8B << 24 | 0x21<< 16 | 0x00 << 8 | 0xFF, 3, true);

        set_interrupt_handler(interrupt, mmce_interrupt_handler);

        unmask_interrupts(OS_INTERRUPTMASK(interrupt) & 
                        (OS_INTERRUPTMASK_EXI_0_TC | OS_INTERRUPTMASK_EXI_1_TC | OS_INTERRUPTMASK_EXI_2_TC));
        exi_dma_read(read_buffer, SECTOR_SIZE, false);

        //DEBUG("WAIT EXI\n");
    }
}

static void mmce_start_read(s32 chan, OSContext *context) {	
    if (exi_channel >= EXI_CHANNEL_MAX)
        __builtin_trap();

    mmce_in_progress = true;
    if (mmce_op) {
        if (!EXILock(exi_channel, exi_device, mmce_start_read))
            return;
        exi_regs[0] |= 0x2; // Reset IRQ
        OSInterrupt interrupt = OS_INTERRUPT_EXI_0_EXI + (3 * exi_channel);

        unmask_interrupts(OS_INTERRUPTMASK(interrupt) & 
                    (OS_INTERRUPTMASK_EXI_0_EXI | OS_INTERRUPTMASK_EXI_1_EXI | OS_INTERRUPTMASK_EXI_2_EXI));

        mmce_state = MMCE_WAIT_EXI;
        set_interrupt_handler(interrupt, mmce_interrupt_handler);
        exi_select();
        exi_imm_write(0x8B << 24 | 0x20 << 16 | ((mmce_op->sector >> 24) & 0xFF) << 8 | ((mmce_op->sector >> 16) & 0xFF), 4, true);
        exi_imm_write(((mmce_op->sector >> 8) & 0xFF) << 24 | (mmce_op->sector & 0xFF) << 16 | ((mmce_op->count >> 8) & 0xFF) << 8 | ((mmce_op->count) & 0xFF), 4, true);
        exi_deselect();

    }
}




bool do_read_write_async(void *buffer, uint32_t length, uint32_t offset, uint64_t sector, bool write, frag_callback callback) {
    mmce_t op;

    op.sector = (uint32_t)sector + (uint32_t)(offset / SECTOR_SIZE);
    op.offset = offset % SECTOR_SIZE;
    op.count = (length + SECTOR_SIZE - 1 + op.offset) / SECTOR_SIZE;
    op.length = length;
    op.buffer = buffer;
    op.user_buffer = buffer;
    op.done_callback = callback;
    op.write = write;
    op.current_block = 0;


    intToStr(op.sector, text);
    DEBUG("SR:");
    DEBUG(text);
    
    memset(text, 0, sizeof(text));
    intToStr(op.count, text);
    DEBUG(" C:");
    DEBUG(text);
    
//    memset(text, 0, sizeof(text));
//    intToStr(op.length, text);
//    DEBUG(" L:");
//    DEBUG(text);
    
//    memset(text, 0, sizeof(text));
//    intToStr(op.offset, text);
//    DEBUG(" O:");
//    DEBUG(text);

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

