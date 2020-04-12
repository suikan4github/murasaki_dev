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

/* -------------------- PLATFORM Type and classes -------------------------- */
#define CHANNEL_LEN 48

/* -------------------- PLATFORM VARIABLES-------------------------- */

// Essential definition.
// Do not delete
murasaki::Platform murasaki::platform;
murasaki::Debugger *murasaki::debugger;

/* ------------------------ PERIPHERAL ----------------------------- */

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
#if 0
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart2;
#endif
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockB1;
/* -------------------- PLATFORM ALGORITHM ------------------------- */

void TaskBodyFunction(const void *ptr);

void InitPlatform()
{
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
    murasaki::platform.i2cMaster = new murasaki::I2cMaster(&hi2c1);
    MURASAKI_ASSERT(nullptr != murasaki::platform.i2cMaster)

    // audio CODEC
    murasaki::platform.codec = new murasaki::Adau1361(
                                                      48000,
                                                      12000000,
                                                      murasaki::platform.i2cMaster,
                                                      0x38);

    murasaki::platform.audioAdapter = new murasaki::SaiPortAdapter(
                                                                   &hsai_BlockB1,
                                                                   &hsai_BlockA1);
    murasaki::platform.audio = new murasaki::DuplexAudio(
                                                         murasaki::platform.audioAdapter,
                                                         CHANNEL_LEN);

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
    murasaki::platform.i2cMaster = new murasaki::I2cMaster(&hi2c1);
    murasaki::platform.i2cSlave = new murasaki::I2cSlave(&hi2c2);
    // For demonstration of master and slave SPI
    murasaki::platform.spiMaster = new murasaki::SpiMaster(&hspi1);
    murasaki::platform.spiSlave = new murasaki::SpiSlave(&hspi4);
#endif

    murasaki::InitCycleCounter();

}

void ExecPlatform()
{
    murasaki::platform.st0->Clear();
    murasaki::platform.st1->Set();

    I2cSearch(murasaki::platform.i2cMaster);

    murasaki::platform.task1->Start();

    while (true)  // dummy loop
    {
        murasaki::platform.led->Toggle();  // toggling LED
        murasaki::Sleep(static_cast<murasaki::WaitMilliSeconds>(700));

    }
}

/* -------------------------- GPIO ---------------------------------- */

/**
 * @brief Optional interrupt handling of EXTI
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param GPIO_Pin Pin number from 0 to 31
 * @details
 * This is called from inside of HAL when an EXTI is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default error interrupt call back.
 *
 * The GPIO_Pin is the number of Pin. For example, if a programmer set the pin name by CubeIDE as FOO, the
 * macro to identify that EXTI is FOO_Pin
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
                            {
#if 0
    // Sample of the EXTI call back.
    // USER_Btn is a standard name of the user push button switch of the Nucleo F722.
    // This switch can be configured as EXTI interrupt srouce.
    // In this sample, releasing the waiting task if interrupt comes.

    // Check whether appropriate interrupt or not
    if ( USER_Btn_Pin == GPIO_Pin) {
        // Check whether sync object is ready or not.
        // This check is essential from the interrupt before the platform variable is ready
        if (murasaki::platform.sync_with_button != nullptr)
        // release the waiting task
            murasaki::platform.sync_with_button->Release();
    }
#endif
}

/* ------------------ User Function -------------------------- */
// Task body of the murasaki::platform.task1
void TaskBodyFunction(const void *ptr) {
    // phase of the oscillator.
    // Let's make phase [0,36000) representing [0,360).

    // audio buffer
    float *l_tx = new float[CHANNEL_LEN];
    float *r_tx = new float[CHANNEL_LEN];
    float *l_rx = new float[CHANNEL_LEN];
    float *r_rx = new float[CHANNEL_LEN];

    murasaki::platform.codec->Start();

    murasaki::SetSyslogFacilityMask(murasaki::kfaAudioCodec);
    murasaki::SetSyslogSererityThreshold(murasaki::kseDebug);

    int count = 0;
    murasaki::platform.audio->TransmitAndReceive(l_tx, r_tx, l_rx, r_rx);

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

        if (count > 10) {
            // disable debug message printing
            murasaki::SetSyslogSererityThreshold(murasaki::kseError);
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
