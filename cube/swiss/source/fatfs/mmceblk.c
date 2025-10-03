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



#define PAGE_SIZE512 512
static bool __mmce_init = false;
static lwpq_t __mmce_queue = LWP_TQUEUE_NULL;

static volatile bool __mmce_io_in_progress = false;
static sem_t __mmce_irq_sem;


static s32 __MMCE_EXI_Handler(s32 chan, s32 dev) {
    if (chan < 0 || chan > 1) return -1;
    
    __mmce_io_in_progress = false;
    LWP_SemPost(__mmce_irq_sem);

    return 1;
}


static u32 mmce_getDevice(s32 chan) {
    if (chan < 0 || chan > 1) return -1;

    return chan == 0 ? DEV_MCE0 : DEV_MCE1; 
}


static bool __mmce_startup(DISC_INTERFACE* disc) {
   /* s32 ret,chan = (disc->ioType&0xff)-'0';
	u32 dev;

	if(disc->ioType < DEVICE_TYPE_GAMECUBE_MCE(0)) return false;
	if(disc->ioType > DEVICE_TYPE_GAMECUBE_MCE(1)) return false;

	if(!__mmce_init) {
        // Initialize the memory card emulator 
		sdgecko_initBufferPool();
		sdgecko_initIODefault();
		__mmce_init = true;
	}

	dev = mmce_getDevice(chan);

	ret = sdgecko_preIO(chan);
	if(ret != CARDIO_ERROR_READY) {
		if(dev == EXI_DEVICE_0)
			sdgecko_setDevice(chan, EXI_DEVICE_2);
		else
			sdgecko_setDevice(chan, EXI_DEVICE_0);

		ret = sdgecko_preIO(chan);
	}

	if(ret == CARDIO_ERROR_READY) {
		dev = sdgecko_getDevice(chan);

		if(chan == EXI_CHANNEL_0) {
			if(dev == EXI_DEVICE_0) {
				disc->features |= FEATURE_GAMECUBE_SLOTA;
				disc->features &= ~FEATURE_GAMECUBE_PORT1;
			} else {
				disc->features |= FEATURE_GAMECUBE_PORT1;
				disc->features &= ~FEATURE_GAMECUBE_SLOTA;
			}
		}

		if(PERM_WRITE_PROTECT(chan) || TMP_WRITE_PROTECT(chan))
			disc->features &= ~FEATURE_MEDIUM_CANWRITE;
		else
			disc->features |= FEATURE_MEDIUM_CANWRITE;

		switch(CSD_STRUCTURE(chan)) {
			case 0:
				disc->numberOfSectors = ((C_SIZE(chan) + 1) << (C_SIZE_MULT(chan) + 2)) << (READ_BL_LEN(chan) - 9);
				break;
			case 1:
				disc->numberOfSectors = (C_SIZE1(chan) + 1LL) << 10;
				break;
			case 2:
				disc->numberOfSectors = (C_SIZE2(chan) + 1LL) << 10;
				break;
			default:
				disc->numberOfSectors = ~0;
				break;
		}

		return true;
	}

	sdgecko_setDevice(chan, dev);*/
    if (!__mmce_init) __mmce_init = true;

    LWP_InitQueue(&__mmce_queue);
    LWP_SemInit(&__mmce_irq_sem, 0, 1);

	return true;
}

static bool __mmce_isInserted(DISC_INTERFACE* disc) 
{
    s32 chan = (disc->ioType&0xff)-'0';
	u32 dev = (chan == 0) ? EXI_DEVICE_0 : EXI_DEVICE_1;
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
	u32 dev = (chan == 0) ? EXI_DEVICE_0 : EXI_DEVICE_1;

    u64 start_time = gettime();

    //print_debug("MMCE read: sector %lld, numSectors %lld\n", sector, numSectors);
    
	if(disc->ioType < DEVICE_TYPE_GAMECUBE_MCE(0)) return false;
	if(disc->ioType > DEVICE_TYPE_GAMECUBE_MCE(1)) return false;

    //print_debug("MMCE read: chan %d, dev %d\n", chan, dev);

    cmd[2] = (sector >> 24) & 0xFF;
    cmd[3] = (sector >> 16) & 0xFF;
    cmd[4] = (sector >> 8) & 0xFF;
    cmd[5] = sector & 0xFF;
    cmd[6] = (numSectors >> 24) & 0xFF;
    cmd[7] = (numSectors >> 16) & 0xFF;
    cmd[8] = (numSectors >> 8) & 0xFF;
    cmd[9] = numSectors & 0xFF;

    //print_debug("MMCE read: locking\n");
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
    __mmce_io_in_progress = true;

	ret |= !EXI_ImmEx(chan, &cmd, sizeof(cmd), EXI_WRITE);
    //print_debug("MMCE read: command sent\n");
	ret |= !EXI_Deselect(chan);

    EXI_Unlock(chan);
    EXI_RegisterEXICallback(chan, __MMCE_EXI_Handler);

    struct timespec timeout = {
        .tv_sec = 5,  // 5 second timeout
        .tv_nsec = 0
    };
    if (LWP_SemTimedWait(__mmce_irq_sem, &timeout) != 0) {
        print_debug("MMCE read: timeout waiting for interrupt\n");
        __mmce_io_in_progress = false;
        return false;
    }

    //LWP_ThreadSleep(__mmce_queue);

    //print_debug("MMCE: Interrupt received, starting read\n");
    

    // Wait for interrupt
    uint8_t retries = 5U;
    
    cmd[1] = 0x21; // Change to read command
    for (sec_t i = 0; i < numSectors; i++) {
        EXI_RegisterEXICallback(chan, __MMCE_EXI_Handler);

        EXI_LockEx(chan, dev);
        u8* ptr = (u8*)buffer + (i * PAGE_SIZE512);
        if (!EXI_Select(chan, dev, EXI_SPEED32MHZ)) 
        {
            EXI_Unlock(chan);
            print_debug("MMCE read: select failed\n");
            return false;
        }
        __mmce_io_in_progress = true;
        EXI_ImmEx(chan, cmd, 2, EXI_WRITE);
        
        ret |= !EXI_DmaEx(chan, ptr, PAGE_SIZE512, EXI_READ);
        ret |= !EXI_Deselect(chan);
        EXI_Unlock(chan);
       // print_debug("Sleeping now... ret: %d\n", ret);
        struct timespec timeout = {
            .tv_sec = 5,  // 5 second timeout
            .tv_nsec = 0
        };
        if (LWP_SemTimedWait(__mmce_irq_sem, &timeout) != 0) {
            print_debug("MMCE read: timeout waiting for interrupt\n");
            __mmce_io_in_progress = false;
            if (--retries > 0U) {
                i -= 1;
                sleep(2);
            }
            else
                break;
        }


    }
    //print_debug("MMCE read: done %lld\n", sector );
    EXI_RegisterEXICallback(chan, NULL);
    u64 time_diff = diff_usec(start_time, gettime());

    print_debug("MMCE read: total read of %lld sectors done in %u ms - %.02f kB/s\n", numSectors, (u32)time_diff / 1000, (float)((numSectors * PAGE_SIZE512 * 1000)  / (time_diff)));

	return ret == 0;
}

static bool __mmce_writeSectors(DISC_INTERFACE* disc, sec_t sector, sec_t numSectors, const void* buffer) {
    u8 cmd[10] = {0x8B, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    
    s32 ret = 0,chan = (disc->ioType&0xff)-'0';
	u32 dev = (chan == 0) ? EXI_DEVICE_0 : EXI_DEVICE_1;

    u64 start_time = gettime();

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
    __mmce_io_in_progress = true;

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
        __mmce_io_in_progress = false;
        return false;
    }

    // Wait for interrupt
    cmd[1] = 0x23; // Change to write command
    for (sec_t i = 0; i < numSectors; i++) {
        EXI_RegisterEXICallback(chan, __MMCE_EXI_Handler);

        EXI_LockEx(chan, dev);
        u8* ptr = (u8*)buffer + (i * PAGE_SIZE512);
        if (!EXI_Select(chan, dev, EXI_SPEED16MHZ)) 
        {
            EXI_Unlock(chan);
            print_debug("MMCE write: select failed\n");
            return false;
        }
        __mmce_io_in_progress = true;
        EXI_ImmEx(chan, cmd, 2, EXI_WRITE);
        
        ret |= !EXI_DmaEx(chan, ptr, PAGE_SIZE512, EXI_WRITE);
        ret |= !EXI_Deselect(chan);
        EXI_Unlock(chan);

        struct timespec timeout = {
            .tv_sec = 5,  // 5 second timeout
            .tv_nsec = 0
        };


        if (LWP_SemTimedWait(__mmce_irq_sem, &timeout) != 0) {
            print_debug("MMCE read: timeout waiting for interrupt\n");
            __mmce_io_in_progress = false;
            break;
        }
    }
    EXI_RegisterEXICallback(chan, NULL);
    u64 time_diff = diff_usec(start_time, gettime());
    
    print_debug("MMCE write: total write of %lld sectors done in %u ms - %.02f kB/s\n", numSectors, (u32)time_diff / 1000, (float)((numSectors * PAGE_SIZE512 * 1000)  / (time_diff)));


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
	64 * 1024 * 1024 * 2, // 64 gb
	PAGE_SIZE512
};

DISC_INTERFACE __io_mmce1 = {
	DEVICE_TYPE_GAMECUBE_MCE(1),
	FEATURE_MEDIUM_CANREAD | /*FEATURE_MEDIUM_CANWRITE |*/ FEATURE_GAMECUBE_SLOTB,
	__mmce_startup,
	__mmce_isInserted,
	__mmce_readSectors,
	__mmce_writeSectors,
	__mmce_clearStatus,
	__mmce_shutdown,
	0,
	PAGE_SIZE512
};
