//
// kernel.h -- mt32-pi-shaped-skel kernel
//
// Bare Circle skeleton: timer, logger (null device -- the SD stats
// file is this kernel's only output channel), scheduler, EMMC + SD
// mount, GPIO manager, I2C master (MiSTer link on core 1 + the sound
// device's DAC-probe master), and the shaped-skel app. No serial,
// no SPI, no USB, no network. Audio leaves via the Pi's own I2S
// device.
//

#ifndef _kernel_h
#define _kernel_h

#include <circle_stdlib_app.h>
#include <circle/gpiomanager.h>
#include <circle/i2cmaster.h>
#include <circle/sched/scheduler.h>
#include <circle/timer.h>

#include "minimal.h"

class CKernel : public CStdlibApp
{
public:
	CKernel(void);

	virtual bool Initialize(void) override;
	TShutdownMode Run(void) override;

protected:
	CTimer m_Timer;
	CLogger m_Logger;
	CScheduler m_Scheduler;
	CEMMCDevice m_EMMC;
	FATFS m_SDFileSystem;
	CGPIOManager m_GPIOManager;
	CI2CMaster m_I2CMaster;

private:
	CMinimalApp m_MinimalApp;
};

#endif
