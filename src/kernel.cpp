//
// kernel.cpp -- mt32-pi-shaped-skel kernel
//

#include "kernel.h"

CKernel::CKernel(void)
	: CStdlibApp("mt32-shaped-skel"),
	  m_Timer(&mInterrupt),
	  m_Logger(mOptions.GetLogLevel(), &m_Timer),
	  m_EMMC(&mInterrupt, &m_Timer, 0),  // no disk-access LED: core 2 heartbeat owns ACT (togglefix)
	  m_SDFileSystem{},
	  m_GPIOManager(&mInterrupt),
	  m_I2CMaster(1, true),
	  m_MinimalApp(&m_I2CMaster, &mInterrupt)
{
}

bool CKernel::Initialize(void)
{
	if (!CStdlibApp::Initialize())
		return false;

	// No UART log in this kernel: the SD stats file is the only
	// output channel (the operator reads the card afterward, same workflow
	// as the mt32-pi diag logs). The logger goes to the null device.
	if (!m_Logger.Initialize(&mNullDevice))
		return false;

	if (!m_Timer.Initialize())
		return false;

	if (!m_EMMC.Initialize())
		return false;

	if (f_mount(&m_SDFileSystem, "SD:", 1) != FR_OK)
	{
		m_Logger.Write(GetKernelName(), LogError, "Failed to mount SD card");
		return false;
	}

	// Init GPIO manager (the hardware keys need it; the GPIO pin
	// objects are created later, in the app's key loop).
	if (!m_GPIOManager.Initialize())
		return false;

	// Init I2C (mt32-pi's setup exactly: device 1, fast mode; don't
	// bother with Initialize() as it only sets the clock to
	// 100/400KHz). No config file in this kernel, so the 400000
	// default is used explicitly. The master serves the MiSTer link
	// (core 1 poll) and the sound device's DAC probe (core 2 Start);
	// Circle serializes its transactions with a TASK_LEVEL spinlock.
	m_I2CMaster.SetClock(400000);

	if (!m_MinimalApp.Initialize())
		return false;

	return true;
}

CStdlibApp::TShutdownMode CKernel::Run(void)
{
	// Never returns: the key loop runs until power cut.
	m_MinimalApp.Run(0);

	return ShutdownHalt;
}
