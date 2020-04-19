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

// Include the prototype  of functions of this file.

/* -------------------- PLATFORM Macros -------------------------- */

/* -------------------- PLATFORM Type and classes -------------------------- */

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
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim2;
extern ADC_HandleTypeDef hadc1;

/* -------------------- PLATFORM Prototypes ------------------------- */

void TaskBodyFunction(const void *ptr);

/* -------------------- PLATFORM Implementation ------------------------- */

void InitPlatform()
{
#if ! MURASAKI_CONFIG_NOCYCCNT
    // Start the cycle counter to measure the cycle in MURASAKI_SYSLOG.
    murasaki::InitCycleCounter();
#endif
    // UART device setting for console interface.
    // On Nucleo, the port connected to the USB port of ST-Link is
    // referred here.
    murasaki::platform.uart_console = new murasaki::DebuggerUart(&huart2);
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

    // For demonstration, one GPIO LED port is reserved.
    // The port and pin names are fined by CubeIDE.
    murasaki::platform.led = new murasaki::BitOut(LD2_GPIO_Port, LD2_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led)

    // Testing the Quadrapture encoder.
    murasaki::platform.encoder = new murasaki::QuadratureEncoder(&htim2);
    MURASAKI_ASSERT(nullptr != murasaki::platform.encoder)

    // For demonstration of FreeRTOS task.
    murasaki::platform.task1 = new murasaki::SimpleTask(
                                                        "task1",
                                                        256,
                                                        murasaki::ktpNormal,
                                                        nullptr,
                                                        &TaskBodyFunction
                                                        );
    MURASAKI_ASSERT(nullptr != murasaki::platform.task1)

    murasaki::platform.adc = new murasaki::Adc(&hadc1);
    MURASAKI_ASSERT(nullptr != murasaki::platform.adc)

    // Following block is just for sample.

}

void ExecPlatform()
{
    // counter for the demonstration.
    unsigned int count = 0;

    // Following blocks are sample.
    murasaki::platform.task1->Start();

    murasaki::platform.adc->SetSampleClock(ADC_CHANNEL_TEMPSENSOR, ADC_SAMPLETIME_56CYCLES);

    // Loop forever
    while (true) {
        count = murasaki::platform.encoder->Get();

        float value;
        murasaki::platform.adc->Convert(ADC_CHANNEL_TEMPSENSOR, &value);

        unsigned int temp = value * 1000;
        murasaki::debugger->Printf("Temperature : %d, timer counter : %d \n", temp, count);

        // wait for a while
        murasaki::Sleep(300);
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
void TaskBodyFunction(const void *ptr)
                      {

    while (true)  // dummy loop
    {
        murasaki::platform.led->Toggle();  // toggling LED
        murasaki::Sleep(700);
    }
}
