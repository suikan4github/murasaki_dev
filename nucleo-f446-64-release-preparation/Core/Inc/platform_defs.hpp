/**
 * \file platform_defs.hpp
 *
 * \date 2018/01/16
 * \author Seiichi "Suikan" Horie
 * \brief Murasaki platform customize file.
 * \details
 * This file contains user defined struct murasaki::Platform.
 *
 * This file will be included by \ref murasaki.hpp.
 *
 */

#ifndef PLATFORM_DEFS_HPP_
#define PLATFORM_DEFS_HPP_

namespace murasaki {
/**
 * @ingroup MURASAKI_PLATFORM_GROUP
 * \{
 */
/**
 * \brief Custom aggregation struct for user platform.
 * \details
 * A collection of the peripheral / MPU control variable.
 *
 * This is a custom struct. Programmer can change this struct as suitable to the hardware and software.
 * But debugger_ member variable have to be left untouched.
 *
 * In the run time, the debugger_ variable have to be initialized by appropriate \ref murasaki::Debugger class instance.
 *
 * See \ref murasaki::platform
 */
struct Platform
{
    // Platform dependent Custom variables.
    UartStrategy *uart_console;    ///< UART wrapping class object for debugging
    LoggerStrategy *logger;        ///< logging class object for debugger

    BitOutStrategy *led1;           ///< GP out under test
    BitOutStrategy *led2;           ///< GP out under test
    BitOutStrategy *led3;           ///< GP out under test
    BitOutStrategy *led4;           ///< GP out under test
    TaskStrategy *master_task;           ///< Task under test
    TaskStrategy *slave_task;           ///< Task under test

    Synchronizer *sync_command;
    Synchronizer *sync_ack;
    // Following block is just sample

    I2cMasterStrategy *i2c_master;  ///< I2C Master under test
    I2cSlaveStrategy *i2c_slave;    ///< I2C Slave under test

    UartStrategy *uart;            ///< UART under test

    SpiMasterStrategy *spi_master;  ///< SPI Master under test
    SpiSlaveStrategy *spi_slave;    ///< SPI Slave under test
    SpiSlaveAdapterStrategy *slave_adapter;

    int test_state = 0;
    bool test_success = false;

};

/**
 * \brief Grobal variable to provide the access to the platform component.
 * \details
 * This variable is declared by murasaki platform. But not instantiated. To make it happen,
 * programmer have to make an variable and initilize it explicitly.
 *
 * Note that the instantiation of this variable is optional. This is provided just of
 * ease of read.
 *
 */
extern Platform platform;

/**
 * \}
 * end of defgroup PLATFORM_GROUP
 */

} /* namespace murasaki */

#endif /* PLATFORM_DEFS_HPP_ */
