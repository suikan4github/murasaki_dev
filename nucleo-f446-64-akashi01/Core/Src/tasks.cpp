#include <string.h>

#include <murasaki.hpp>
#include "murasaki_platform.hpp"

static const int i2c_device = 0x39;

void Test001Master(void);
void Test001Slave(void);
void Test101Master(void);
void Test101Slave(void);
void Test201Master(void);
void Test201Slave(void);

static const int cmd_001 = 1;
static const int cmd_101 = 101;
static const int cmd_201 = 201;



/* ------------------ User Functions -------------------------- */
/**
 * @brief Master test task.
 * @param ptr Pointer to the parameter block
 * @details
 * Task body function of the test.
 * Call test subprogram step by step.
 *
 */
void MasterTaskBodyFunction(const void *ptr) {

    murasaki::platform.led1->Clear();  // toggling LED

    Test001Master();
    Test101Master();
    Test201Master();

#if 1
    // Display text on the LED module.
    murasaki::platform.i2c_master->Transmit(
                                            0x71,
                                            reinterpret_cast<const uint8_t*>("Fin."),
                                            4);
#endif
    // end of test. blink LED1
    while (true)
    {
        murasaki::Sleep(1000);
        murasaki::platform.led1->Toggle();
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
void SlaveTaskBodyFunction(const void *ptr) {

    while (true)
    {
        murasaki::platform.sync_command->Wait();

        switch (
        murasaki::platform.test_state) {
            case cmd_001:
                Test001Slave();
                break;
            case cmd_101:
                Test101Slave();
                break;
            case cmd_201:
                Test201Slave();
                break;
            default:
                murasaki::debugger->Printf("Unknown command \n");
                MURASAKI_ASSERT(false)
                ;
    	}

    }
}

/* ****************************************************************************************
 *                              Test 201
 * SPI master TX, slave RX normal test.
 * The master send a data to the slave.
 * The TX length and the RX length are identical.
 */

static uint8_t tx_data_spi_master[] = "Hello, SPI";
static uint8_t rx_data_spi_master[30] = "*********************";

static uint8_t tx_data_spi_slave[30] = "The quick brown fox ";
static uint8_t rx_data_spi_slave[30] = "*********************";


void Test201Master(void) {
    // Start SPI simple test.
    murasaki::platform.test_state = cmd_201;
    murasaki::platform.test_success = false;

    murasaki::platform.sync_command->Release();     // tell slave task next test

    murasaki::platform.spi_master->TransmitAndReceive(
                                                      murasaki::platform.slave_adapter,
                                                      tx_data_spi_master,
                                                      rx_data_spi_master,
                                                      sizeof(tx_data_spi_master));
                                     

    murasaki::platform.sync_ack->Wait();
    murasaki::debugger->Printf(
                               "test %03d : %s \n",
                               cmd_201,
                               murasaki::platform.test_success ?
                                                                 "Good" :
                                                                 "NG");

}

void Test201Slave(void) {

    murasaki::platform.spi_slave->TransmitAndReceive(
                                                     tx_data_spi_slave,
                                                     rx_data_spi_slave,
                                                     sizeof(tx_data_spi_master));
    murasaki::platform.test_success =
            0 == strncmp(
                         reinterpret_cast<char*>(tx_data_spi_master),
                         reinterpret_cast<char*>(rx_data_spi_slave),
                         sizeof(tx_data_spi_master));
    murasaki::platform.sync_ack->Release();

}

/* ****************************************************************************************
 *                              Test 101
 * UART TX, RX normal test.
 * The TX send a data to the RX.
 * The TX length and the RX length are identical.
 */

static uint8_t tx_data_uart[] = "Hello, UART";
static uint8_t rx_data_uart[30] = "*********************";

void Test101Master(void) {
    // Start UART simple test.
    murasaki::platform.test_state = cmd_101;
    murasaki::platform.test_success = false;

    murasaki::platform.sync_command->Release();     // tell slave task next test

    murasaki::platform.uart->Transmit(
                                      tx_data_uart,
                                      sizeof(tx_data_uart)
                                      );

    murasaki::platform.sync_ack->Wait();
    murasaki::debugger->Printf(
                               "test %03d : %s \n",
                               cmd_101,
                               murasaki::platform.test_success ?
                                                                 "Good" :
                                                                 "NG");

}

void Test101Slave(void) {

    murasaki::platform.uart->Receive(
                                     rx_data_uart,
                                     sizeof(tx_data_uart));
    murasaki::platform.test_success =
            0 == strncmp(
                         reinterpret_cast<char*>(tx_data_uart),
                         reinterpret_cast<char*>(rx_data_uart),
                         sizeof(tx_data_uart));
    murasaki::platform.sync_ack->Release();

}

/* ****************************************************************************************
 *                              Test 001
 * I2C Master Slave normal tels.
 * The master send a data to the slave.
 * The master TX length and the slave RX length are identical.
 */

static uint8_t tx_data_i2c[] = "Hello, I2C";
static uint8_t rx_data_i2c[30] = "*********************";

void Test001Master(void) {
    unsigned int transfered_count;

    // Start I2C simple test.
    murasaki::platform.test_state = cmd_001;
    murasaki::platform.test_success = false;

    murasaki::platform.sync_command->Release();     // tell slave task next test

    murasaki::platform.i2c_master->Transmit(
                                            i2c_device,
                                            tx_data_i2c,
                                            sizeof(tx_data_i2c),
                                            &transfered_count);

    murasaki::platform.sync_ack->Wait();
    murasaki::debugger->Printf(
                               "test %03d : %s \n",
                               cmd_001,
                               murasaki::platform.test_success ?
                                                                 "Good" :
                                                                 "NG");

}

void Test001Slave(void) {
    unsigned int transfered_count;

    memset(
           rx_data_i2c,
           0,
           sizeof(rx_data_i2c));

    murasaki::platform.i2c_slave->Receive(
                                          rx_data_i2c,
                                          sizeof(tx_data_i2c),
                                          &transfered_count);
    murasaki::platform.test_success =
            0 == strncmp(
                         reinterpret_cast<char*>(tx_data_i2c),
                         reinterpret_cast<char*>(rx_data_i2c),
                         sizeof(tx_data_i2c));
    murasaki::platform.sync_ack->Release();

}

