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
#include "testsi5351.hpp"

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

//@formatter:off
inline uint32_t synth_p1(uint32_t a, uint32_t b, uint32_t c) { return 128 * a + (128 * b) / c - 512; }
inline uint32_t synth_p2(uint32_t a, uint32_t b, uint32_t c) { return 128*b + c*((128 * b) / c); }
inline uint32_t synth_p3(uint32_t a, uint32_t b, uint32_t c) { return c; }

inline uint8_t msynth_reg0(uint32_t p1, uint32_t p2, uint32_t p3) { return (p3 >> 8) & 0xFF; }
inline uint8_t msynth_reg1(uint32_t p1, uint32_t p2, uint32_t p3) { return p3 & 0xFF; }
inline uint8_t msynth_reg2(uint32_t p1, uint32_t p2, uint32_t p3, uint32_t r_div, uint32_t divby4) { return ((r_div & 0x07) << 4) | ((divby4 & 0x03) << 2) | ((p1 >> 16) & 0x03); }
inline uint8_t msynth_reg3(uint32_t p1, uint32_t p2, uint32_t p3) { return (p1 >> 8) & 0xFF; }
inline uint8_t msynth_reg4(uint32_t p1, uint32_t p2, uint32_t p3) { return p1 & 0xFF; }
inline uint8_t msynth_reg5(uint32_t p1, uint32_t p2, uint32_t p3) { return (((p3 >> 16) & 0x0F ) << 4 ) | ((p2 >> 16) & 0x0F); }
inline uint8_t msynth_reg6(uint32_t p1, uint32_t p2, uint32_t p3) { return (p2 >> 8) & 0xFF; }
inline uint8_t msynth_reg7(uint32_t p1, uint32_t p2, uint32_t p3) { return p2 & 0xFF; }
//@formatter:on

// main routine of the system.
void ExecPlatform()
{
// counter for the demonstration.
    int count = 0;

    murasaki::I2cSearch(murasaki::platform.i2c_master);

#if 0
    // Set PLL 36 times ( 25MHz * PLL = 900MHz)
    uint32_t a = 36;
    uint32_t b = 0;
    uint32_t c = 1;

    uint32_t p1 = synth_p1(a, b, c);
    uint32_t p2 = synth_p2(a, b, c);
    uint32_t p3 = synth_p3(a, b, c);

    uint8_t pll_value[8] = {
            msynth_reg0(p1, p2, p3),
            msynth_reg1(p1, p2, p3),
            msynth_reg2(p1, p2, p3, 0, 0),
            msynth_reg3(p1, p2, p3),
            msynth_reg4(p1, p2, p3),
            msynth_reg5(p1, p2, p3),
            msynth_reg6(p1, p2, p3),
            msynth_reg7(p1, p2, p3)
    };

    // Set Multi-synth a + b/c
    a = 10;
    b = 0;
    c = 1;

    p1 = synth_p1(a, b, c);
    p2 = synth_p2(a, b, c);
    p3 = synth_p3(a, b, c);

    uint8_t msynth_value[8] = {
            msynth_reg0(p1, p2, p3),
            msynth_reg1(p1, p2, p3),
            msynth_reg2(p1, p2, p3, 0, 0),  // R0_DIV = div by 1,
            msynth_reg3(p1, p2, p3),
            msynth_reg4(p1, p2, p3),
            msynth_reg5(p1, p2, p3),
            msynth_reg6(p1, p2, p3),
            msynth_reg7(p1, p2, p3)
    };

    murasaki::Si5351ClockControl clockConfig;


    // Reading Device Status
    murasaki::debugger->Printf("PLL reg 0 : 0x%02x\n", murasaki::platform.pll->GetRegister(0));
    // Reading PLL input source
    murasaki::debugger->Printf("PLL reg 15 : 0x%02x\n", murasaki::platform.pll->GetRegister(15));
    // Check Clock 0 control
    murasaki::debugger->Printf("PLL reg 16 : 0x%02x\n", murasaki::platform.pll->GetRegister(16));
#if 0
    // While the datasheet directs to disable the output, we don't need it.
    // Output Disable
    murasaki::debugger->Printf("PLL reg 3 : 0x%02x\n", murasaki::platform.pll->GetRegister(3));
    murasaki::platform.pll->SetRegister(3, 0x01);  // Disable clock output)
    murasaki::debugger->Printf("PLL reg 3 : 0x%02x\n", murasaki::platform.pll->GetRegister(3));
#endif

    // Power down divider
    clockConfig.fields.clk_idrv = murasaki::ks5351od4mA;
    clockConfig.fields.clk_inv = false;
    clockConfig.fields.clk_pdn = true;
    clockConfig.fields.clk_src = murasaki::ks5351osNativeDivider;
//    clockConfig.fields.clk_src = murasaki::ks5351osXtal;
    clockConfig.fields.ms_int = true;
    clockConfig.fields.ms_src = murasaki::ks5351PllA;

#if 0
    // Power down of the PLL before setting, is optional.
    murasaki::platform.pll->SetClockConfig(0, clockConfig);
#endif
    // Check Clock 0 control
    murasaki::debugger->Printf("PLL reg 16 : 0x%02x\n", murasaki::platform.pll->GetRegister(16));

    // Set PLLA
    murasaki::debugger->Printf("Setting PLLA by writing to reg 26\n");
    murasaki::platform.pll->SetRegister(26, pll_value, 8);

    // Set multi-synth
    murasaki::debugger->Printf("Setting Multi-Synth0 by writing to reg 42\n");
    murasaki::platform.pll->SetRegister(42, msynth_value, 8);

    // Power up divider
    clockConfig.fields.clk_pdn = false;
    murasaki::platform.pll->SetClockConfig(0, clockConfig);

    // reset PLL
    murasaki::debugger->Printf("Writing reg 177 to reset PLL A&B\n");
    murasaki::platform.pll->SetRegister(177, 0xa0);  // clock0, output disable )
#if 0
    // Output Enable
    murasaki::debugger->Printf("PLL reg 3 : 0x%02x\n", murasaki::platform.pll->GetRegister(3));
    murasaki::platform.pll->SetRegister(3, 0x00);  // Enable clock output)
    murasaki::debugger->Printf("PLL reg 3 : 0x%02x\n", murasaki::platform.pll->GetRegister(3));
#endif
    // Check Clock 0 control
    murasaki::debugger->Printf("PLL reg 16 : 0x%02x\n", murasaki::platform.pll->GetRegister(16));
    // Check MS0 R_DIV settings
    murasaki::debugger->Printf("PLL reg 44 : 0x%02x\n", murasaki::platform.pll->GetRegister(44));

#else
// Set frequency for test
//@formatter:off
#if 0
    unsigned int freq = 2500;
    murasaki::debugger->Printf("PLL Single output test : %dHz \n", freq);

    murasaki::platform.pll->SetFrequency(
                                    murasaki::ks5351PllA,
                                         0,
                                    freq);
#else
    unsigned int freq = 50490000;
    murasaki::Si5351ClockControl clockConfig;

    murasaki::debugger->Printf("PLL Quadrature output test : %dHz \n", freq);

    // Power down divider
    clockConfig.fields.clk_idrv = murasaki::ks5351od4mA;
    clockConfig.fields.clk_pdn = true;

    // set the output drive power
    murasaki::platform.pll->SetClockConfig(0, clockConfig);
    murasaki::platform.pll->SetClockConfig(1, clockConfig);

    // Quadrature operation
    murasaki::platform.pll->SetQuadratureFrequency(
                                    murasaki::ks5351PllA,
                                         0,
                                         1,
                                    freq);
#endif                                                                                                                                                                                                                                                                                                                                                             //@formatter:on
    murasaki::platform.task1->Start();
#endif

    murasaki::SetSyslogSeverityThreshold(murasaki::kseNotice);
    murasaki::TestSi5351Driver(100);

// Loop forever
    while (true) {

        // print a message with counter value to the console.
        murasaki::debugger->Printf("Hello %d \n", count);

        // update the counter value.
        count++;

        // wait for a while
        murasaki::Sleep(3000);
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

