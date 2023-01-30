/**
 * @file Energesis_SHT31_Temp.cpp
 * @author Jose Guerra Carmenate(joseguerracarmenate@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Energesis_SHT31.hpp"

namespace Energesis{

bool Energesis_SHT31_Temp::getSample( sensor_sample_t* sample ){

  sample->type = SENSOR_TYPE_TEMPERATURE;
  sample->sensor_id = 0;
  sample->temperature = m_sht31->getTemperature();
  sample->timestamp = millis();

  if( sample->temperature == NAN )
    return false;

  return true;
}

void Energesis_SHT31_Temp::getSensorDetails( sensor_details_t* details ){
  details->max_value = 125;
  details->min_delay = 0;
  details->min_value = -40;
  strlcpy(details->name, "SHT31", sizeof("SHT31"));
  details->power = 0.002; // 2uA

  details->resolution = 0.015;
  details->sensor_id = 0;
  details->type = SENSOR_TYPE_TEMPERATURE;
  //details->version = ENERGESIS_SHT2X_VERSION_MAJOR;
}

};
