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
void I2cSearch(murasaki::I2CMasterStrategy *master);

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

    murasaki::platform.audioAdapter = new murasaki::SaiPortAdaptor(
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

/* ------------------------- UART ---------------------------------- */

/**
 * @brief Essential to sync up with UART.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param huart
 * @details
 * This is called from inside of HAL when an UART transmission done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default TX interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::TransmissionCompleteCallback() function.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
                             {
    // Poll all uart tx related interrupt receivers.
    // If hit, return. If not hit,check next.
    if (murasaki::platform.uart_console->TransmitCompleteCallback(huart))
        return;

}

/**
 * @brief Essential to sync up with UART.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param huart
 * @details
 * This is called from inside of HAL when an UART receive done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default RX interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::ReceiveCompleteCallback() function.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
                             {
    // Poll all uart rx related interrupt receivers.
    // If hit, return. If not hit,check next.
    if (murasaki::platform.uart_console->ReceiveCompleteCallback(huart))
        return;

}

/**
 * @brief Optional error handling of UART
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param huart
 * @details
 * This is called from inside of HAL when an UART error interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default error interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::HandleError() function.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // Poll all uart error related interrupt receivers.
    // If hit, return. If not hit,check next.
    if (murasaki::platform.uart_console->HandleError(huart))
        return;

}

/* -------------------------- SPI ---------------------------------- */

#ifdef HAL_SPI_MODULE_ENABLED

/**
 * @brief Essential to sync up with SPI.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hspi
 * @details
 * This is called from inside of HAL when an SPI transfer done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default TX/RX interrupt call back.
 *
 * In this call back, the SPI device handle have to be passed to the
 * murasaki::Spi::TransmitAndReceiveCompleteCallback () function.
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Poll all SPI TX RX related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 0
     if ( murasaki::platform.spi1->TransmitAndReceiveCompleteCallback(hspi) )
     return;
#endif
}

/**
 * @brief Optional error handling of SPI
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hspi
 * @details
 * This is called from inside of HAL when an SPI error interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default error interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::HandleError() function.
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef * hspi) {
    // Poll all SPI error interrupt related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 0
     if ( murasaki::platform.spi1->HandleError(hspi) )
     return;
#endif
}

#endif

/* -------------------------- I2C ---------------------------------- */

#ifdef HAL_I2C_MODULE_ENABLED

/**
 * @brief Essential to sync up with I2C.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C transmission done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default TX interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::I2c::TransmitCompleteCallback() function.
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
                                  {
    // Poll all I2C master tx related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 1
    if (murasaki::platform.i2cMaster->TransmitCompleteCallback(hi2c))
        return;
#endif
}

/**
 * @brief Essential to sync up with I2C.
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C receive done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default RX interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::Uart::ReceiveCompleteCallback() function.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Poll all I2C master rx related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 1
    if (murasaki::platform.i2cMaster->ReceiveCompleteCallback(hi2c))
        return;
#endif
}
/**
 * @brief Essential to sync up with I2C.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C transmission done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default TX interrupt call back.
 *
 * In this call back, the I2C slave device handle have to be passed to the
 * murasaki::I2cSlave::TransmitCompleteCallback() function.
 */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
                                 {
    // Poll all I2C master tx related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 0
    if (murasaki::platform.i2c_slave->TransmitCompleteCallback(hi2c))
    return;
#endif
}

/**
 * @brief Essential to sync up with I2C.
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C receive done interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default RX interrupt call back.
 *
 * In this call back, the I2C slave device handle have to be passed to the
 * murasaki::I2cSlave::ReceiveCompleteCallback() function.
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    // Poll all I2C master rx related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 0
    if (murasaki::platform.i2c_slave->ReceiveCompleteCallback(hi2c))
    return;
#endif
}

/**
 * @brief Optional error handling of I2C
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hi2c
 * @details
 * This is called from inside of HAL when an I2C error interrupt is accepted.
 *
 * STM32Cube HAL has same name function internally.
 * That function is invoked whenever an relevant interrupt happens.
 * In the other hand, that function is declared as weak bound.
 * As a result, this function overrides the default error interrupt call back.
 *
 * In this call back, the uart device handle have to be passed to the
 * murasaki::I2c::HandleError() function.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    // Poll all I2C master error related interrupt receivers.
    // If hit, return. If not hit,check next.
#if 1
    if (murasaki::platform.i2cMaster->HandleError(hi2c))
        return;
#endif
}

#endif

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

/* ------------------ SAI  -------------------------- */
#ifdef HAL_SAI_MODULE_ENABLED
/**
 * @brief Optional SAI interrupt handler at buffer transfer halfway.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the SAI device.
 * @details
 * Invoked after SAI RX DMA complete interrupt is at halfway.
 * This interrupt have to be forwarded to the  murasaki::DuplexAudio::ReceiveCallback().
 * The second parameter of the ReceiveCallback() have to be 0 which mean the halfway interrupt.
 */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    if (murasaki::platform.audio->DmaCallback(hsai, 0)) {
        murasaki::platform.st0->Set();
        murasaki::platform.st1->Clear();
        return;
    }
}

/**
 * @brief Optional SAI interrupt handler at buffer transfer complete.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the SAI device.
 * @details
 * Invoked after SAI RX DMA complete interrupt is at halfway.
 * This interrupt have to be forwarded to the  murasaki::DuplexAudio::ReceiveCallback().
 * The second parameter of the ReceiveCallback() have to be 1 which mean the complete interrupt.
 */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
    if (murasaki::platform.audio->DmaCallback(hsai, 1)) {
        murasaki::platform.st0->Clear();
        murasaki::platform.st1->Set();
        return;
    }
}

/**
 * @brief Optional SAI error interrupt handler.
 * @ingroup MURASAKI_PLATFORM_GROUP
 * @param hsai Handler of the SAI device.
 * @details
 * The error have to be forwarded to murasaki::DuplexAudio::HandleError().
 * Note that DuplexAudio::HandleError() trigger a hard fault.
 * So, never return.
 */

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
    if (murasaki::platform.audio->HandleError(hsai))
        return;
}

#endif

/* ------------------ ASSERTION AND ERROR -------------------------- */

void CustomAssertFailed(uint8_t *file, uint32_t line)
                        {
    murasaki::debugger->Printf("Wrong parameters value: file %s on line %d\n",
                               file,
                               line);
    // To stop the execusion, raise assert.
    MURASAKI_ASSERT(false);
}

void CustomDefaultHandler() {
    // Call debugger's post mortem processing. Never return again.
    murasaki::debugger->DoPostMortem();
}

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName) {
    murasaki::debugger->Printf("Stack overflow at task :  %s \n", pcTaskName);
    MURASAKI_ASSERT(false);
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

    }

}

void I2cSearch(murasaki::I2CMasterStrategy *master)
               {
    uint8_t tx_buf[1];

    murasaki::debugger->Printf("            Probing I2C devices \n");
    murasaki::debugger->Printf("   | 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
    murasaki::debugger->Printf("---+------------------------------------------------\n");

    // Search raw
    for (int raw = 0; raw < 128; raw += 16) {
        // Search column
        murasaki::debugger->Printf("%2x |", raw);
        for (int col = 0; col < 16; col++) {
            murasaki::I2cStatus result;
            // check whether device exist or not.
            result = master->Transmit(raw + col, tx_buf, 0);
            if (result == murasaki::ki2csOK)  // device acknowledged.
                murasaki::debugger->Printf(" %2X", raw + col);
            else if (result == murasaki::ki2csNak)  // no device
                murasaki::debugger->Printf(" --");
            else
                murasaki::debugger->Printf(" ??");  // unpredicted error.
        }
        murasaki::debugger->Printf("\n");
    }

}
