/**
 * @file murasaki_platform.cpp
 *
 * @date 2018/05/20
 * @author Seiichi "Suikan" Horie
 * @brief A glue file between the user application and HAL/RTOS.
 */

// Include the definition created by CubeIDE.
#include <murasaki_platform.hpp>
#include "callbackrepositorysingleton.hpp"
#include "main.h"

// Include the murasaki class library.
#include "murasaki.hpp"
#include "murasaki_syslog.hpp"

// Include the prototype  of functions of this file.
#include <math.h>
/* -------------------- PLATFORM Macros -------------------------- */

/* -------------------- PLATFORM Type and classes -------------------------- */
#define CHANNEL_LEN 48

/* -------------------- PLATFORM Variables-------------------------- */

// Essential definition.
// Do not delete
murasaki::Platform murasaki::platform;
murasaki::Debugger *murasaki::debugger;

/* ------------------------ STM32 Peripherals ----------------------------- */

/*
 * Platform dependent peripheral declaration.
 *
 * The variables here are defined at the top of the main.c.
 * Only the variable needed by the InitPlatform() are declared here
 * as external symbols.
 *
 * The declaration here is user project dependent.
 */
// Following block is just sample.
// Original declaration is in the top of main.c.
#if 0
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart2;
#endif
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;

extern I2S_HandleTypeDef hi2s1;
/* -------------------- PLATFORM Prototypes ------------------------- */

void TaskBodyFunction(const void *ptr);

/* -------------------- PLATFORM Implementation ------------------------- */

// Initialization of the asystem. 
void InitPlatform()
{
#if ! MURASAKI_CONFIG_NOCYCCNT
    // Start the cycle counter to measure the cycle in MURASAKI_SYSLOG.
    murasaki::InitCycleCounter();
#endif
    // UART device setting for console interface.
    // On Nucleo, the port connected to the USB port of ST-Link is
    // referred here.
    murasaki::platform.uart_console = new murasaki::DebuggerUart(&huart3);
    while (nullptr == murasaki::platform.uart_console)
        ;  // stop here on the memory allocation failure.

    // UART is used for logging port.
    // At least one logger is needed to run the debugger class.
    murasaki::platform.logger = new murasaki::UartLogger(murasaki::platform.uart_console);
    while (nullptr == murasaki::platform.logger)
        ;  // stop here on the memory allocation failure.

    // Setting the debugger
    murasaki::debugger = new murasaki::Debugger(murasaki::platform.logger);
    while (nullptr == murasaki::debugger)
        ;  // stop here on the memory allocation failure.

    // Set the debugger as AutoRePrint mode, for the easy operation.
    murasaki::debugger->AutoRePrint();  // type any key to show history.


    // Setup I2C.
    murasaki::platform.i2c_master = new murasaki::I2cMaster(&hi2c1);
    MURASAKI_ASSERT(nullptr != murasaki::platform.i2c_master)

    // audio CODEC
    murasaki::platform.codec = new murasaki::Adau1361(
                                                      48000,
                                                      12000000,
                                                      murasaki::platform.i2c_master,
                                                      0x38);
#if 0
    murasaki::platform.audio_adapter = new murasaki::SaiPortAdaptor(
                                                                   &hsai_BlockB1,
                                                                   &hsai_BlockA1);
#else
    murasaki::platform.audio_adapter = new murasaki::I2sPortAdapter(
                                                                    &hi2s1
                                                                    );
#endif
    MURASAKI_ASSERT(nullptr != murasaki::platform.audio_adapter)

    murasaki::platform.audio = new murasaki::DuplexAudio(
                                                         murasaki::platform.audio_adapter,
                                                         CHANNEL_LEN);
    MURASAKI_ASSERT(nullptr != murasaki::platform.audio)

    // Status LED
    murasaki::platform.st0 = new murasaki::BitOut(ST0_GPIO_Port, ST0_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.st0)
    murasaki::platform.st1 = new murasaki::BitOut(ST1_GPIO_Port, ST1_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.st1)

    // For demonstration, one GPIO LED port is reserved.
    // The port and pin names are fined by CubeIDE.
    murasaki::platform.led = new murasaki::BitOut(LD2_GPIO_Port, LD2_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led)

    // For demonstration of FreeRTOS task.
    murasaki::platform.task1 = new murasaki::SimpleTask(
                                                        "task1",
                                                        2048, /* Stack size*/
                                                        murasaki::ktpRealtime,
                                                        nullptr, /* Stack must be allocated by system */
                                                        &TaskBodyFunction
                                                        );
    MURASAKI_ASSERT(nullptr != murasaki::platform.task1)
    // Following block is just for sample.
#if 0
    // For demonstration of the serial communication.
    murasaki::platform.uart = new murasaki::Uart(&huart2);
    // For demonstration of master and slave I2C
    murasaki::platform.i2c_master = new murasaki::I2cMaster(&hi2c1);
    murasaki::platform.i2c_slave = new murasaki::I2cSlave(&hi2c2);
    // For demonstration of master and slave SPI
    murasaki::platform.spi_master = new murasaki::SpiMaster(&hspi1);
    murasaki::platform.spi_slave = new murasaki::SpiSlave(&hspi4);
#endif

}

// main routine of the system.
void ExecPlatform()
{
    murasaki::platform.st0->Clear();
    murasaki::platform.st1->Set();

    I2cSearch(murasaki::platform.i2c_master);

    murasaki::Sleep(500);

    murasaki::debugger->Printf("DWT_CTRL : %08x \n", DWT->CTRL);

    murasaki::platform.task1->Start();

    while (true)  // dummy loop
    {
        murasaki::platform.led->Toggle();  // toggling LED
        murasaki::Sleep(static_cast<murasaki::WaitMilliSeconds>(700));

    }
}
/* ------------------ User Functions -------------------------- */
/**
 * @brief Demonstration task.
 * @param ptr Pointer to the parameter block
 * @details
 * Task body function as demonstration of the @ref murasaki::SimpleTask.
 *
 * You can delete this function if you don't use.
 */
void TaskBodyFunction(const void *ptr){
// phase of the oscillator.
// Let's make phase [0,36000) representing [0,360).

	// audio buffer
	float *l_tx = new float[CHANNEL_LEN];
	float *r_tx = new float[CHANNEL_LEN];
	float *l_rx = new float[CHANNEL_LEN];
	float *r_rx = new float[CHANNEL_LEN];

	murasaki::platform.codec->Start();

#if 0
	murasaki::SetSyslogFacilityMask(murasaki::kfaI2s | murasaki::kfaAudio);
	murasaki::SetSyslogSererityThreshold(murasaki::kseDebug);
#endif

	int count = 0;

	murasaki::platform.codec->SetGain(
									  murasaki::kccHeadphoneOutput,
									  0.0,
									  0.0);  // right gain in dB, left gain in dB
	murasaki::platform.codec->SetGain(
									  murasaki::kccLineInput,
									  0.0,
									  0.0);  // right gain in dB, left gain in dB
	murasaki::platform.codec->SetGain(
									  murasaki::kccLineOutput,
									  0.0,
									  0.0);

	murasaki::platform.codec->Mute(
								   murasaki::kccHeadphoneOutput,
								   false
								   );
	murasaki::platform.codec->Mute(
								   murasaki::kccLineInput,
								   false
								   );
	murasaki::platform.codec->Mute(
								   murasaki::kccLineOutput,
								   false
								   );

	// Loop forever
	while (true) {

		if (count > 4) {
			// disable debug message printing
			murasaki::SetSyslogSeverityThreshold(murasaki::kseError);
		}
		else
			count++;

		// Talk through
		for (int i = 0; i < CHANNEL_LEN; i++) {
			l_tx[i] = l_rx[i];
			r_tx[i] = r_rx[i];
		}
		murasaki::platform.audio->TransmitAndReceive(l_tx, r_tx, l_rx, r_rx);

		murasaki::platform.st0->Toggle();
		murasaki::platform.st1->Toggle();

	}

}

