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
#define MAXDEPTH 32
#define YPICSIZE 55
#define XPICSIZE 100

#define XPOS(x)  ( float(x) / XPICSIZE - 0.5 ) * 4
#define YPOS(y)  ( float(y) / YPICSIZE - 0.5 ) * 4

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
extern UART_HandleTypeDef hlpuart1;

/* -------------------- PLATFORM Prototypes ------------------------- */

unsigned int mandelbrot(float x, float y, int maxn);

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
    murasaki::platform.uart_console = new murasaki::DebuggerUart(&hlpuart1);
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

}

unsigned int mandelbrot(float x, float y, int maxn) {
    float a = x;
    float b = y;
    int n = 0;

    while (a * a + b * b < 4.0) {
        // Limit depth by maxn -1
        if (n >= maxn)
            return maxn - 1;

        // Y = Zn^2 + c : where c is x+iy;
        float r = a * a - b * b + x;
        float i = 2 * a * b + y;
        // Update Zn+1 = Y
        a = r;
        b = i;
        // Update N
        n++;
    }

    return n;
}

char mapper[] = " %#822oooo========----------------............................................................ ";

void ExecPlatform()
{

    murasaki::debugger->Printf("\n");
    murasaki::debugger->Printf("    Mandelbrot set by NUCLEO-G431RB\n");

    // Render the set.
    for (int y = 0; y < YPICSIZE; y++) {
        for (int x = 0; x < XPICSIZE; x++)
            // print a message with counter value to the console.
            murasaki::debugger->Printf("%c",
                                       mapper[mandelbrot(XPOS(x),
                                                         YPOS(y),
                                                         sizeof(mapper) - 1)]);
        murasaki::debugger->Printf("\n");
    }
    // Loop forever
    while (true) {
        // Blink LED
        murasaki::platform.led->Toggle();
        // wait for a while
        murasaki::Sleep(500);
    }
}

