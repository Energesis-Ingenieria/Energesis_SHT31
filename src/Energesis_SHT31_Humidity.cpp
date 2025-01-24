/**
 * @file Energesis_SHT31_Humidity.cpp
 * @author Jose Guerra Carmenate (joseguerracarmenate@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Energesis_SHT31.hpp"

namespace Energesis{

bool Energesis_SHT31_Humidity::getSample( sensor_sample_t* sample ){

  if (m_sht31 == nullptr)
    return false;

  sample->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sample->sensor_id = 0;
  sample->humidity = m_sht31->getRelativeHumidity();
  sample->timestamp = millis();

  if( sample->humidity == NAN )
    return false;

  return true;
}

void Energesis_SHT31_Humidity::getSensorDetails( sensor_details_t* details ){
  details->max_value = 100;
  details->min_delay = 0;
  details->min_value = 0;
  strlcpy(details->name, "SHT31", sizeof("SHT31"));
  details->power = 0.002; // 2uA

  details->resolution = 0.01;
  details->sensor_id = 0;
  details->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  //details->version = ENERGESIS_SHT2X_VERSION_MAJOR;
}

}
