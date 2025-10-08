#include "mmceblk.h"
#include "diskio.h"
#include "dvd.h"
#include "gctypes.h"
#include "ogc/exi.h"
#include "ogc/lwp.h"
#include "ogc/semaphore.h"
#include "ogc/timesupp.h"
#include "settings.h"
#include "util.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/unistd.h>



#define MMCE_PAGE_SIZE512 512

#define PARTITION_TABLE_OFFSET 0x1BE
#define PARTITION_ENTRY_SIZE   16
#define NUM_PARTITIONS         4

static sem_t __mmce_irq_sem;

static bool __mmce_readSectors(DISC_INTERFACE* disc, sec_t sector, sec_t numSectors, void* buffer);


static s32 __MMCE_EXI_Handler(s32 chan, s32 dev) {
    if (chan < 0 || chan > 1) return -1;
    
    LWP_SemPost(__mmce_irq_sem);

    return 1;
}


static bool __mmce_startup(DISC_INTERFACE* disc) {
    /*
    s32 ret,chan = (disc->ioType&0xff)-'0';
    u32 dev;
    
    if(disc->ioType < DEVICE_TYPE_GAMECUBE_MCE(0)) return false;
    if(disc->ioType > DEVICE_TYPE_GAMECUBE_MCE(1)) return false;
    u8 mbr [MMCE_PAGE_SIZE512] = {};
    dev = EXI_DEVICE_0;

    // Read MBR (0x00, 512B)
    __mmce_readSectors(disc, 0x00, 1U, mbr);
    // for each entry (16Byte) check 0x08 (4Byte le) + 0x0C (4Byte le)

    for (int i = 0; i < NUM_PARTITIONS; i++) {
        const uint8_t *entry = mbr + PARTITION_TABLE_OFFSET + i * PARTITION_ENTRY_SIZE;

        uint32_t start_lba = (uint32_t)(entry + 8)[0]
                            | ((uint32_t)(entry + 8)[1] << 8)
                            | ((uint32_t)(entry + 8)[2] << 16)
                            | ((uint32_t)(entry + 8)[3] << 24);
        uint32_t num_sectors = (uint32_t)(entry + 12)[0]
                            | ((uint32_t)(entry + 12)[1] << 8)
                            | ((uint32_t)(entry + 12)[2] << 16)
                            | ((uint32_t)(entry + 12)[3] << 24);

        if (num_sectors == 0)
            continue; // empty partition entry

        uint32_t end_lba = start_lba + num_sectors; // one past the last sector
        if (end_lba > disc->numberOfSectors);
            disc->numberOfSectors = end_lba;
    }
*/

    LWP_SemInit(&__mmce_irq_sem, 0, 1);

    return true;
}

static bool __mmce_isInserted(DISC_INTERFACE* disc) 
{
    s32 chan = (disc->ioType&0xff)-'0';
    u32 dev =  EXI_DEVICE_0;
    if (EXI_Probe(chan))
    {
        bool err = false;
        u8 cmd[2];
        u32 id = 0x00;

        if (!EXI_LockEx(chan, dev)) return false;
        if (!EXI_Select(chan, dev, EXI_SPEED16MHZ)) {
            EXI_Unlock(chan);
            return false;
        }

        cmd[0] = 0x8B;
        cmd[1] = 0x00;

        err |= !EXI_ImmEx(chan, cmd, sizeof(cmd), EXI_WRITE);
        err |= !EXI_ImmEx(chan, &id, sizeof(id), EXI_READ);
        err |= !EXI_Deselect(chan);
        EXI_Unlock(chan);
        print_debug("MMCE read: id %08x\n", id);
        print_debug("Error: %d\n", err);
        if (err)
            return false;
        else if (id >> 16 != 0x3842)
            return false;
        else
            return true;
    }
    return false;
}

static bool __mmce_readSectors(DISC_INTERFACE* disc, sec_t sector, sec_t numSectors, void* buffer) {
    u8 cmd[10] = {0x8B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    s32 ret = 0,chan = (disc->ioType&0xff)-'0';
    u32 dev = EXI_DEVICE_0;
    
    if(disc->ioType < DEVICE_TYPE_GAMECUBE_MCE(0)) return false;
    if(disc->ioType > DEVICE_TYPE_GAMECUBE_MCE(1)) return false;

    cmd[2] = (sector >> 24) & 0xFF;
    cmd[3] = (sector >> 16) & 0xFF;
    cmd[4] = (sector >> 8) & 0xFF;
    cmd[5] = sector & 0xFF;
    cmd[6] = (numSectors >> 24) & 0xFF;
    cmd[7] = (numSectors >> 16) & 0xFF;
    cmd[8] = (numSectors >> 8) & 0xFF;
    cmd[9] = numSectors & 0xFF;

    if (EXI_LockEx(chan, dev) < 0)
    {
        print_debug("MMCE read: lock failed\n");
        return false;
    }
    
    if (!EXI_Select(chan, dev, EXI_SPEED16MHZ)) 
    {
        EXI_Unlock(chan);
        return false;
    }

    ret |= !EXI_ImmEx(chan, &cmd, sizeof(cmd), EXI_WRITE);
    ret |= !EXI_Deselect(chan);

    EXI_Unlock(chan);
    EXI_RegisterEXICallback(chan, __MMCE_EXI_Handler);

    struct timespec timeout = {
        .tv_sec = 5,  // 5 second timeout
        .tv_nsec = 0
    };
    if (LWP_SemTimedWait(__mmce_irq_sem, &timeout) != 0) {
        print_debug("MMCE read: timeout waiting for interrupt\n");
        return false;
    }


    // Wait for interrupt
    uint8_t retries = 5U;
    
    cmd[1] = 0x21; // Change to read command
    for (sec_t i = 0; i < numSectors; i++) {
        EXI_RegisterEXICallback(chan, __MMCE_EXI_Handler);

        EXI_LockEx(chan, dev);
        u8* ptr = (u8*)buffer + (i * MMCE_PAGE_SIZE512);
        if (!EXI_Select(chan, dev, EXI_SPEED32MHZ)) 
        {
            EXI_Unlock(chan);
            print_debug("MMCE read: select failed\n");
            return false;
        }
        EXI_ImmEx(chan, cmd, 2, EXI_WRITE);
        
        ret |= !EXI_DmaEx(chan, ptr, MMCE_PAGE_SIZE512, EXI_READ);
        ret |= !EXI_Deselect(chan);
        EXI_Unlock(chan);
        struct timespec timeout = {
            .tv_sec = 5,  // 5 second timeout
            .tv_nsec = 0
        };
        if (LWP_SemTimedWait(__mmce_irq_sem, &timeout) != 0) {
            print_debug("MMCE read: timeout waiting for interrupt\n");
            if (--retries > 0U) {
                i -= 1;
                sleep(2);
            }
            else
                break;
        }
    }
    EXI_RegisterEXICallback(chan, NULL);

    return ret == 0;
}

static bool __mmce_writeSectors(DISC_INTERFACE* disc, sec_t sector, sec_t numSectors, const void* buffer) {
    u8 cmd[10] = {0x8B, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    s32 ret = 0,chan = (disc->ioType&0xff)-'0';
    u32 dev = EXI_DEVICE_0;

    if(disc->ioType < DEVICE_TYPE_GAMECUBE_MCE(0)) return false;
    if(disc->ioType > DEVICE_TYPE_GAMECUBE_MCE(1)) return false;

    cmd[2] = (sector >> 24) & 0xFF;
    cmd[3] = (sector >> 16) & 0xFF;
    cmd[4] = (sector >> 8) & 0xFF;
    cmd[5] = sector & 0xFF;
    cmd[6] = (numSectors >> 24) & 0xFF;
    cmd[7] = (numSectors >> 16) & 0xFF;
    cmd[8] = (numSectors >> 8) & 0xFF;
    cmd[9] = numSectors & 0xFF;

    if (EXI_LockEx(chan, dev) < 0)
    {
        print_debug("MMCE read: lock failed\n");
        return false;
    }
    
    if (!EXI_Select(chan, dev, EXI_SPEED16MHZ)) 
    {
        EXI_Unlock(chan);
        return false;
    }

    ret |= !EXI_ImmEx(chan, &cmd, sizeof(cmd), EXI_WRITE);
    ret |= !EXI_Deselect(chan);

    EXI_Unlock(chan);
    EXI_RegisterEXICallback(chan, __MMCE_EXI_Handler);

    struct timespec timeout = {
        .tv_sec = 5,  // 5 second timeout
        .tv_nsec = 0
    };
    if (LWP_SemTimedWait(__mmce_irq_sem, &timeout) != 0) {
        print_debug("MMCE write: timeout waiting for interrupt\n");
        return false;
    }

    // Wait for interrupt
    cmd[1] = 0x23; // Change to write command
    for (sec_t i = 0; i < numSectors; i++) {
        EXI_RegisterEXICallback(chan, __MMCE_EXI_Handler);

        EXI_LockEx(chan, dev);
        u8* ptr = (u8*)buffer + (i * MMCE_PAGE_SIZE512);
        if (!EXI_Select(chan, dev, EXI_SPEED16MHZ)) 
        {
            EXI_Unlock(chan);
            print_debug("MMCE write: select failed\n");
            return false;
        }
        EXI_ImmEx(chan, cmd, 2, EXI_WRITE);
        
        ret |= !EXI_DmaEx(chan, ptr, MMCE_PAGE_SIZE512, EXI_WRITE);
        ret |= !EXI_Deselect(chan);
        EXI_Unlock(chan);

        struct timespec timeout = {
            .tv_sec = 5,  // 5 second timeout
            .tv_nsec = 0
        };


        if (LWP_SemTimedWait(__mmce_irq_sem, &timeout) != 0) {
            print_debug("MMCE read: timeout waiting for interrupt\n");
            break;
        }
    }
    EXI_RegisterEXICallback(chan, NULL);

    return ret == 0;
}

static bool __mmce_clearStatus(DISC_INTERFACE* disc) {
    return false;
}

static bool __mmce_shutdown(DISC_INTERFACE* disc) {
    return false;
}



DISC_INTERFACE __io_mmce0 = {
    DEVICE_TYPE_GAMECUBE_MCE(0),
    FEATURE_MEDIUM_CANREAD | FEATURE_MEDIUM_CANWRITE | FEATURE_GAMECUBE_SLOTA,
    &__mmce_startup,
    &__mmce_isInserted,
    &__mmce_readSectors,
    &__mmce_writeSectors,
    &__mmce_clearStatus,
    &__mmce_shutdown,
    0xFFFFFFFFFFFFFFFF,
    MMCE_PAGE_SIZE512
};

DISC_INTERFACE __io_mmce1 = {
    DEVICE_TYPE_GAMECUBE_MCE(1),
    FEATURE_MEDIUM_CANREAD | FEATURE_MEDIUM_CANWRITE | FEATURE_GAMECUBE_SLOTB,
    __mmce_startup,
    __mmce_isInserted,
    __mmce_readSectors,
    __mmce_writeSectors,
    __mmce_clearStatus,
    __mmce_shutdown,
    0xFFFFFFFFFFFFFFFF,
    MMCE_PAGE_SIZE512
};
