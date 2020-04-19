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
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/* -------------------- PLATFORM Prototypes ------------------------- */

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
    murasaki::platform.led1 = new murasaki::BitOut(LED1_GPIO_Port, LED1_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led1)
    murasaki::platform.led2 = new murasaki::BitOut(LED2_GPIO_Port, LED2_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led2)
    murasaki::platform.led3 = new murasaki::BitOut(LED3_GPIO_Port, LED3_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led3)
    murasaki::platform.led4 = new murasaki::BitOut(LED4_GPIO_Port, LED4_Pin);
    MURASAKI_ASSERT(nullptr != murasaki::platform.led4)

    // For demonstration of FreeRTOS task.
    murasaki::platform.master_task = new murasaki::SimpleTask(
                                                              "MasterTask",
                                                              256,
                                                              murasaki::ktpNormal,
                                                              nullptr,
                                                              &MasterTaskBodyFunction
                                                              );
    MURASAKI_ASSERT(nullptr != murasaki::platform.master_task)

    murasaki::platform.slave_task = new murasaki::SimpleTask(
                                                             "SlaveTask",
                                                             256,
                                                             murasaki::ktpHigh, /* slave has higher priority.*/
                                                             nullptr,
                                                             &SlaveTaskBodyFunction
                                                             );

    MURASAKI_ASSERT(nullptr != murasaki::platform.slave_task)

    murasaki::platform.sync_command = new murasaki::Synchronizer();
    MURASAKI_ASSERT(nullptr != murasaki::platform.sync_command)

    murasaki::platform.sync_ack = new murasaki::Synchronizer();
    MURASAKI_ASSERT(nullptr != murasaki::platform.sync_ack)

    murasaki::platform.i2c_master = new murasaki::I2cMaster(&hi2c1);
    MURASAKI_ASSERT(nullptr != murasaki::platform.i2c_master)

    murasaki::platform.i2c_slave = new murasaki::I2cSlave(&hi2c2);
    MURASAKI_ASSERT(nullptr != murasaki::platform.i2c_slave)

    murasaki::platform.spi_master = new murasaki::SpiMaster(&hspi2);
    MURASAKI_ASSERT(nullptr != murasaki::platform.spi_master)

    murasaki::platform.spi_slave = new murasaki::SpiSlave(&hspi1);
    MURASAKI_ASSERT(nullptr != murasaki::platform.spi_slave)

    // CPOL and CPHA follows pin configration of SPI 1.
    murasaki::platform.slave_adapter = new murasaki::SpiSlaveAdapter(
                                                                     0,
                                                                     0,
                                                                     SPI_CS_GPIO_Port,
                                                                     SPI_CS_Pin);

    murasaki::platform.uart = new murasaki::Uart(&huart1);
    MURASAKI_ASSERT(nullptr != murasaki::platform.uart)

    // Following block is just for sample.

}

void ExecPlatform()
{
    // counter for the demonstration.
    int count = 0;

    I2cSearch(murasaki::platform.i2c_master);

    murasaki::platform.master_task->Start();
    murasaki::platform.slave_task->Start();

    // Loop forever
    while (true) {

        // update the counter value.
        count++;

        murasaki::platform.led4->Toggle();  // toggling LED
        murasaki::Sleep(700);
    }
}
