/**
 *  @file    AQI.h
 *  @author  peter c
 *  @date    11/22/2021
 *  @version 0.1
 *
 *
 *  @section DESCRIPTION
 *  Simple calculation map ug/m3 to a US Air Quality Index
 *   https://en.wikipedia.org/wiki/Air_quality_index
 *
 */
#ifndef AQI_H
#define AQI_H
#include <stdint.h>

// US EPA AQI Health Level Concerns
typedef enum
{
    GOOD = 0,
    MODERATE,
    UNHEALTHY_SENSITVE_GROUPS,
    UNHEALTHY,
    VERY_HUNHEALTHY,
    HAZARDOUS,
    UNKNOWN

} AQI_Category;


// AQI calculation return value
typedef struct
{

    float index;
    AQI_Category category;
} AQI_Data;

// AQI equation constant entry
typedef struct
{
    float Clow;
    float Chigh;
    uint16_t Ilow;
    uint16_t Ihigh;
    AQI_Category category;

} AQI_Entry;

class AQI
{
public:
    AQI();
    // return the larger of the two PM10 or PM25
    AQI_Data get_AQI(float pm10Concentration, float pm25Concentration);
    // return US PM10
    AQI_Data get_AQI_PM10(float concentration);
    //return US PM25
    AQI_Data get_AQI_PM25(float concentration);

private:
    // https://en.wikipedia.org/wiki/Air_quality_index
    // PM 10 *microg/m3
    AQI_Entry AQI_US_10[7] = {
        {0.0, 54.0, 0, 50, GOOD},
        {55.0, 154.0, 51, 100, MODERATE},
        {155, 254.0, 101, 150, UNHEALTHY_SENSITVE_GROUPS},
        {255.0, 354.0, 151, 200, UNHEALTHY},
        {355.0, 424.0, 201, 300, VERY_HUNHEALTHY},
        {425.0, 504.0, 301, 400, HAZARDOUS},
        {505.0, 604.0, 401, 500, HAZARDOUS},
    };

    // PM 2.5 *microg/m3
    AQI_Entry AQI_US_25[7] = {
        {0.0, 12.0, 0, 50, GOOD},
        {12.1, 35.4, 51, 100, MODERATE},
        {35.5, 55.4, 101, 150, UNHEALTHY_SENSITVE_GROUPS},
        {55.5, 150.4, 151, 200, UNHEALTHY},
        {150.5, 250.4, 201, 300, VERY_HUNHEALTHY},
        {250.5, 350.4, 301, 400, HAZARDOUS},
        {350.5, 500.4, 401, 500, HAZARDOUS}};

    float calcAQI(float concentration, float Chigh, float Clow, float Ihigh, float Ilow);
    AQI_Data computeAQI(float concentration, AQI_Entry aqi_entries[], uint16_t size);
};

#endif