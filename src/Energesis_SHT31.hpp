/**
 * @file Energesis_SHT31.hpp
 * @author Jose Guerra Carmenate (joseguerracarmenate@gmail.com)
 * @brief Arduino library for the SHT31 temperature and humidity sensor.
 * @version 0.1
 * @date 2022-07-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _ENERGESIS_SHT31_H_
#define _ENERGESIS_SHT31_H_

#include <Energesis_Sensor.h>
#include <Energesis_I2CDevice.h>


#define SHT31_DEFAULT_ADDR 0x44 /**< SHT31 Default Address */


#define SHT31_CMD_HEATER_ENABLE             0x306D  /*!< Command to start internal heater */
#define SHT31_CMD_HEATER_DISABLE            0x3066  /*!< Command to stop internal heater */
#define SHT31_CMD_READ_STATUS               0xF32D  /*!< Command to get status register */
#define SHT31_CMD_CLEAR_STATUS              0xF32D  /*!< Command to clear status register */
#define SHT31_CMD_RESET                     0x30A2  /*!< Command to reset sensor */

/**
 * @name Measurement Commands Single Shot
 * Measurement Commands for Single Shot Data Acquisition Mode.
 */
//@{
#define SHT31_CMD_MEASURE_HIGH_REP_STRETCH  0x2C06  /*!< Command to perform high repeatability measure  (whit stretch) */
#define SHT31_CMD_MEASURE_MED_REP_STRETCH   0x2C0D  /*!< Command to perform medium repeatability measure  (whit stretch) */
#define SHT31_CMD_MEASURE_LOW_REP_STRETCH   0x2C10  /*!< Command to perform low repeatability measure  (whit stretch) */
#define SHT31_CMD_MEASURE_HIGH_REP          0x2400  /*!< Command to perform high repeatability measure */
#define SHT31_CMD_MEASURE_MED_REP           0x240B  /*!< Command to perform medium repeatability measure */
#define SHT31_CMD_MEASURE_LOW_REP           0x2416  /*!< Command to perform low repeatability measure */
//@}


#define SHT31_STATUS_REG_WRITE_CHECKSUM_MASK    0x0001 // see datasheet page 13
#define SHT31_STATUS_REG_CMD_MASK               0x0002 // see datasheet page 13
#define SHT31_STATUS_REG_SYS_RESET_MASK         0x0010 // see datasheet page 13
#define SHT31_STATUS_REG_TEMP_ALERT_MASK        0x0400 // see datasheet page 13
#define SHT31_STATUS_REG_RH_ALERT_MASK          0x0800 // see datasheet page 13
#define SHT31_STATUS_REG_HEATER_MASK            0x2000 // see datasheet page 13
#define SHT31_STATUS_REG_ALERT_PENDING_MASK     0x8000 // see datasheet page 13


namespace Energesis{

class Energesis_SHT31; // forward declaration 

/**
 * @brief Definition of generic temperature sensor for SHT31.
 * 
 */
class Energesis_SHT31_Temp: public Energesis_Sensor{
public:
  Energesis_SHT31_Temp( Energesis_SHT31 *sens ){ m_sht31 = sens; }

  bool getSample(sensor_sample_t *sample) override;

  void getSensorDetails( sensor_details_t* ) override;

private:
  Energesis_SHT31 *m_sht31 = NULL;

};

/**
 * @brief Definition of generic relative humidity sensor for SHT31.
 * 
 */
class Energesis_SHT31_Humidity: public Energesis_Sensor{
public:
  Energesis_SHT31_Humidity( Energesis_SHT31 *sens ){ m_sht31 = sens; }

  bool getSample(sensor_sample_t *sample) override;

  void getSensorDetails( sensor_details_t* ) override;

private:
  Energesis_SHT31 *m_sht31 = NULL;

};


/**
 * @brief SHT31 Driver
 * 
 */
class Energesis_SHT31: public Energesis_TemperatureSensor, public Energesis_RelativeHumiditySensor {    
public:
    Energesis_SHT31(){};
    ~Energesis_SHT31(){};

#if defined(ESP8266) || defined(ESP32)
    /**
     * @brief Initialise controller and I2C bus on specified pins
     * 
     * @note Only for ESP32 and ESP8266 chips
     * 
     * @param[in] sda pin for SDA signal 
     * @param[in] scl pin for SCL signal
     * @param[in] wire pointer to Wire interface to use (default &Wire)
     * @param i2c_address sensor's address on I2C bus
     * @return true Controller and bus initialised
     * @return false Initialisation error
     */
    bool begin( int sda, int scl, TwoWire *wire = &Wire, uint8_t i2c_address = SHT31_DEFAULT_ADDR  );
#endif

    /**
     * @brief Initialise controller and I2C bus on specified pins
     * 
     * @param[in] wire pointer to Wire interface to use (default &Wire)
     * @param i2c_address sensor's address on I2C bus
     * @return true Controller and bus initialised
     * @return false Initialisation error
     */
    bool begin( TwoWire *wire = &Wire, uint8_t i2c_address = SHT31_DEFAULT_ADDR );

    /**
     * @see Energesis_TemperatureSensor::getTemperature()
     */
    float getTemperature() override;

    /**
     * @see Energesis_TemperatureSensor::getTemperatureSensor()
     */
    Energesis_Sensor *getTemperatureSensor() override;

    /**
     * @see Energesis_RelativeHumiditySensor::getRelativeHumidity()
     */
    float getRelativeHumidity() override;

    /**
     * @see Energesis_RelativeHumiditySensor::getRelativeHumiditySensor()
     */
    Energesis_Sensor* getRelativeHumiditySensor() override;

    /**
     * @brief Get status register
     * 
     * @return uint16_t status register
     */
    uint16_t readStatus();

    /**
     * @brief Send a soft reset command to sensor. 
     * 
     * @return true soft reset successful.
     * @return false communication error
     */
    bool reset();

    /**
     * @brief Start or stop internal heater
     * 
     * @param enable true for start and false for stop.
     */
    void heater(bool enable);

    /**
     * @brief Check heater state 
     * 
     * @return true heater is on
     * @return false heater is off
     */
    bool isHeaterOn();

protected:
    Energesis_I2CDevice *m_device = NULL; //!< Communication bus 
    uint8_t m_tx_buff[6];                 //!< transmission buffer 
    uint8_t m_rx_buff[10];                //!< reception buffer

private:
    /**
     * @brief check crc for one byte array
     * 
     * @param[in] arr array 
     * @param[in] len array's length
     * @param crc correct crc value
     * @return true crc match
     * @return false crc mismatch
     */
    bool checkCRC( uint8_t *arr, uint8_t len, uint8_t crc);

    /**
     * @brief Perform a single shot measure.
     * 
     * @return true measure performed correct
     * @return false failed to perform measure
     */
    bool measure();

    float m_temperature, /*!< last temperature measure */
          m_humidity; /*!< last humidity measure */

    bool m_begun;  /*!< begun flag */

    friend class Energesis_SHT31_Temp;
    friend class Energesis_SHT31_Humidity;
};

}

#endif

