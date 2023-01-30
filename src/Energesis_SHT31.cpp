/**
 * @file Energesis_SHT31.cpp
 * @author Jose Guerra Carmenate (joseguerracarmenate@gmail.com)
 * @brief Arduino library for the SHT31 temperature and humidity sensor.
 * @version 0.1
 * @date 2022-07-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Energesis_SHT31.hpp>

using namespace Energesis;

#define DEBUG Serial 

#if defined(ESP8266) || defined(ESP32)
bool Energesis_SHT31::begin( int sda, int scl, TwoWire *wire, uint8_t i2c_address ){
  if( m_begun )
    return false;
  // Crear bus
  if( !m_device )
    m_device = new Energesis_I2CDevice( i2c_address, sda, scl, wire );

  return begin( wire, i2c_address );
}
#endif

bool Energesis_SHT31::begin( TwoWire *wire, uint8_t i2c_address ){

  if( m_begun ){
    DEBUG.println("[E] El sensor ya está inicializado");
    return false;
  }

  // Crear bus
  if( !m_device )
    m_device = new Energesis_I2CDevice( i2c_address, wire );

  if( !m_device ){
    DEBUG.println( "[E] No se creó la instancia de I2CDevice" );
    return false;
  }

  // Inicializar bus
  if( !m_device->begin() ){
    DEBUG.println( "[E] No se inicializó el bus" );
    delete m_device;
    return false;
  }

  // comprobar que esté conectado
  if( !m_device->isConnected() ){
    DEBUG.println( "[E] Dispositivo no detectado" );
    return false;
  }

  if( !reset() ){
    DEBUG.println( "[E] No se pudo reiniciar el sensor" );
    return false;
  }

  m_begun = true;
  return true;
}


float Energesis_SHT31::getTemperature(){
    if( !measure() ){
        return NAN;
    }
    return m_temperature;
}

float Energesis_SHT31::getRelativeHumidity(){
    if( !measure() ){
        return NAN;
    }
    return m_humidity;
}

bool Energesis_SHT31::reset(){
    m_tx_buff[0] = (SHT31_CMD_RESET>>8);
    m_tx_buff[1] = SHT31_CMD_RESET & 0xFF;
    return m_device->write( m_tx_buff, 2 );
}

void Energesis_SHT31::heater(bool enable){
    if( enable ){
        m_tx_buff[0] = (SHT31_CMD_HEATER_ENABLE>>8);
        m_tx_buff[1] = SHT31_CMD_HEATER_ENABLE & 0xFF;
    }
    else {
        m_tx_buff[0] = (SHT31_CMD_HEATER_DISABLE>>8);
        m_tx_buff[1] = SHT31_CMD_HEATER_DISABLE & 0xFF;
    }

    m_device->write(m_tx_buff, 2);
    delay(1);
}

uint16_t Energesis_SHT31::readStatus(){
    uint16_t status;
    m_tx_buff[0] = (SHT31_CMD_READ_STATUS>>8);
    m_tx_buff[1] = SHT31_CMD_READ_STATUS & 0xFF;
    if( !m_device->write( m_tx_buff, 2, false ) ){
        DEBUG.println( "[E] Al obtener registro de estado." );
//        return false;
    }

    m_device->read( m_rx_buff, 3 );

    status = (m_rx_buff[0]<<8) | m_rx_buff[1]; 
    return status;
}

bool Energesis_SHT31::isHeaterOn(){
    return (bool)( readStatus() & SHT31_STATUS_REG_HEATER_MASK );
}

bool Energesis_SHT31::measure(){
  m_tx_buff[0] = SHT31_CMD_MEASURE_HIGH_REP>>8;
  m_tx_buff[1] = SHT31_CMD_MEASURE_HIGH_REP&0xFF;
  m_device->write( m_tx_buff, 2 );
  delay(20);
  m_device->read( m_rx_buff, 6 );

  if ( !checkCRC(m_rx_buff, 2, m_rx_buff[2]) || !checkCRC(m_rx_buff + 3, 2, m_rx_buff[5]))
    return false;
  
  uint32_t raw_temperature = (uint32_t)(m_rx_buff[0]<<8) | m_rx_buff[1];
  uint32_t raw_humidity = (uint32_t)(m_rx_buff[3]<<8) | m_rx_buff[4];

  m_temperature = -45.0 + (175.0*(float)raw_temperature)/(float)((uint32_t)(1<<16) - 1);

  m_humidity = (100.0*(float)raw_humidity)/(float)((uint32_t)(1<<16) - 1);
  return true;
}


Energesis_Sensor *Energesis_SHT31::getTemperatureSensor(){
  if( !m_temperature_sensor )
    m_temperature_sensor = new Energesis_SHT31_Temp( this );
  return m_temperature_sensor;
}

Energesis_Sensor *Energesis_SHT31::getRelativeHumiditySensor(){
  if( !m_humidity_sensor )
    m_humidity_sensor = new Energesis_SHT31_Humidity(this);
  return m_humidity_sensor;
}


const static uint8_t POLY = 0x31;

bool Energesis_SHT31::checkCRC( uint8_t *arr, uint8_t len, uint8_t real_crc ){
  uint8_t crc = 0xFF;

  for (int j = len; j; --j) {
    crc ^= *arr++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
    }
  }
  return (crc == real_crc);
}


