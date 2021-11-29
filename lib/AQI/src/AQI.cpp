/**
 *  @file    AQI.h
 *  @author  peter c
 *  @date    11/22/2021
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Simple calculation map ug/m3 to a US Air Quality Index
 
 *
 */

#include <Streaming.h>

#include "AQI.h"

// PM 10 *microg/m3

#define INRANGE(x, x_min, x_max) ((x) >= (x_min) && (x) <= (x_max))

AQI::AQI()
{
}

float AQI::calcAQI(float concentration, float Chigh, float Clow, float Ihigh, float Ilow)
{
    return (Ihigh - Ilow) * (concentration - Clow) / (Chigh - Clow) + Ilow;
}

AQI_Data AQI::computeAQI(float concentration, AQI_Entry aqi_entries[], uint16_t size)
{
    AQI_Data result = {-1, UNKNOWN};

    for (uint16_t i = 0; i < size; i++)
    {
        if (INRANGE(concentration, aqi_entries[i].Clow, aqi_entries[i].Chigh))
        {
            result.index = calcAQI(concentration, aqi_entries[i].Clow, aqi_entries[i].Chigh, aqi_entries[i].Ilow, aqi_entries[i].Ihigh);
            result.category = aqi_entries[i].category;
            break;
        }
    }
    return result;
}

AQI_Data AQI::get_AQI(float pm10Concentration, float pm25Concentration)
{
    AQI_Data pm10 = get_AQI_PM10(pm10Concentration);
    AQI_Data pm25 = get_AQI_PM25(pm25Concentration);


    if (pm10.index > pm25.index)
        return pm10;

    return pm25;
}

AQI_Data AQI::get_AQI_PM25(float concentration)
{
    AQI_Data result = computeAQI(concentration, AQI_US_25, sizeof(AQI_US_25) / sizeof(AQI_Entry));
    return result;
}

AQI_Data AQI::get_AQI_PM10(float concentration)
{
    AQI_Data result = computeAQI(concentration, AQI_US_10, sizeof(AQI_US_10) / sizeof(AQI_Entry));
    return result;
}