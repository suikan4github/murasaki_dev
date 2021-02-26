/**
* @file murasaki_include_stub.h
* @brief Stub to include the HAL headers. 
* @details
* The CubeIDE add the STM32 microprocessor product name as pre-defined macro when a file is compiled. 
* For example, following is the macro definition for STM32F446 processor at the compiler command line. 
* @code
* -DSTM32F446xx
* @endcode
* 
* On the other hand, this is not enough to determine the appropriate include file inside murasaki_defs.hpp. 
* As a result, there are difficulties to include the appropriate file. 
* 
* One of the naive appropach is to enumulate all possible pre-defined macro to determine the filename as following.
* 
* @code
* #elif defined (STM32F405xx) || defined (STM32F415xx) || defined (STM32F407xx) || defined (STM32F417xx) || *    defined (STM32F427xx) || defined (STM32F437xx) || defined (STM32F429xx) || defined (STM32F439xx) || *    defined (STM32F401xC) || defined (STM32F401xE) || defined (STM32F410Tx) || defined (STM32F410Cx) || *    defined (STM32F410Rx) || defined (STM32F411xE) || defined (STM32F446xx) || defined (STM32F469xx) || *    defined (STM32F479xx) || defined (STM32F412Cx) || defined (STM32F412Rx) || defined (STM32F412Vx) || *    defined (STM32F412Zx) || defined (STM32F413xx) || defined (STM32F423xx)
* #include "stm32f4xx_hal.h"
* @endcode
* 
* This is easy to understand. But boring to maintain. 
* 
* This stub is alternate way. @ref murasaki_defs.hpp is including this file (murasaki_include_stub.h).
* And this stub file include the appropriate HAL header file. 
* This stub file is generated by murasaki/install script. 
* Thus, user doesn't need to maintain this file. 
*/
#include<stm32g4xx_hal.h>
