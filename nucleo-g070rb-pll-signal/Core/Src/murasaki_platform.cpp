/**
 * @file murasaki_platform.cpp
 *
 * @date 2018/05/20
 * @author Seiichi "Suikan" Horie
 * @brief A glue file between the user application and HAL/RTOS.
 */

// Include the definition created by CubeIDE.
#include <murasaki_platform.hpp>
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
// Original declaration is in the top of main.c.
#if 0
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart3;
#endif
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

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
    murasaki::platform.led = new murasaki::BitOut(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led)

    // I2C master port.
    // At least one logger is needed to run the debugger class.
    murasaki::platform.i2c_master = new murasaki::I2cMaster(&hi2c1);
    MURASAKI_ASSERT(nullptr != murasaki::platform.i2c_master)

    // Clock Generator Click Board
    //@formatter:off
    murasaki::platform.pll = new murasaki::Si5351(
                                                  murasaki::platform.i2c_master,    // I2C controller
                                                  0x60,                             // I2C slave address
                                                  25000000                               // 25MHz for Clock Generator Click board
            );
                                                                                                                                                                                                            //@formatter:on

    MURASAKI_ASSERT(nullptr != murasaki::platform.pll)

    // For demonstration of FreeRTOS task.
    murasaki::platform.task1 = new murasaki::SimpleTask(
                                                        "task1",
                                                        256,
                                                        murasaki::ktpNormal,
                                                        nullptr,
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
    // counter for the demonstration.
    int count = 0;
    murasaki::Si5351ClockControl clockConfig;

    murasaki::I2cSearch(murasaki::platform.i2c_master);

    murasaki::debugger->Printf("PLL reg 183 : 0x%02x\n", murasaki::platform.pll->GetRegister(183));
    murasaki::platform.pll->SetRegister(183, 0x92);  // clock0, output disable )
    murasaki::debugger->Printf("PLL reg 183 : 0x%02x\n", murasaki::platform.pll->GetRegister(183));
    murasaki::debugger->Printf("PLL reg 0 : 0x%02x\n", murasaki::platform.pll->GetRegister(0));
    murasaki::debugger->Printf("PLL reg 1 : 0x%02x\n", murasaki::platform.pll->GetRegister(1));
    murasaki::debugger->Printf("PLL reg 2 : 0x%02x\n", murasaki::platform.pll->GetRegister(2));
    murasaki::debugger->Printf("PLL reg 3 : 0x%02x\n", murasaki::platform.pll->GetRegister(3));
    murasaki::platform.pll->SetRegister(3, 0x01);  // clock0, output disable )
    murasaki::debugger->Printf("PLL reg 3 : 0x%02x\n", murasaki::platform.pll->GetRegister(3));

    murasaki::debugger->Printf("PLL reg 16 : 0x%02x\n", murasaki::platform.pll->GetRegister(16));
    murasaki::platform.pll->SetRegister(16, 0x01);  // clock0, power up, xtalin, 4mA )
    murasaki::debugger->Printf("PLL reg 16 : 0x%02x\n", murasaki::platform.pll->GetRegister(16));
    murasaki::debugger->Printf("PLL reg 44 : 0x%02x\n", murasaki::platform.pll->GetRegister(44));
    murasaki::platform.pll->SetRegister(44, 0x00);  // set R0 to 0
    murasaki::debugger->Printf("Writing reg 177 to reset\n");
    murasaki::platform.pll->SetRegister(177, 0xa0);  // clock0, output disable )
    murasaki::platform.pll->SetRegister(3, 0x00);  // clock0, output disable )
    murasaki::debugger->Printf("PLL reg 44 : 0x%02x\n", murasaki::platform.pll->GetRegister(44));
    murasaki::platform.pll->SetRegister(3, 0x00);  // clock0, output disable )
    murasaki::debugger->Printf("PLL reg 3 : 0x%02x\n", murasaki::platform.pll->GetRegister(3));

    clockConfig.fields.clk_idrv = murasaki::ks5351od4mA;
    clockConfig.fields.clk_inv = false;
    clockConfig.fields.clk_pdn = false;
    clockConfig.fields.clk_src = murasaki::ks5351osNativeDivider;
    clockConfig.fields.ms_int = false;
    clockConfig.fields.ms_src = murasaki::ks5351PllA;

    murasaki::platform.pll->SetClockConfig(0, clockConfig);

#if 1
    // Set frequency for test
    //@formatter:off
    murasaki::platform.pll->SetFrequency(
                                    murasaki::ks5351PllA,
                                         0,
                                         100000);
    murasaki::platform.pll->ResetPLL(murasaki::ks5351PllA);
                                                                              //@formatter:on
    murasaki::platform.task1->Start();
#endif
    // Loop forever
    while (true) {

        // print a message with counter value to the console.
        murasaki::debugger->Printf("Hello %d \n", count);

        // update the counter value.
        count++;

        // wait for a while
        murasaki::Sleep(500);
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

