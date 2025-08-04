/*
 * packet.h
 *
 *  Created on: May 7, 2025
 *      Author: Halil
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

#include "bmi088.h"
#include "bme280.h"
#include "l86_gnss.h"
#include "flight_algorithm.h"


typedef union {
    float sayi;
    uint8_t array[4];
} FLOAT32_UINT8_DONUSTURUCU;


unsigned char check_sum_hesapla_normal(int a);


void addDataPacketNormal(BME_280_t* BME, bmi088_struct_t* BMI, gps_data_t* GPS, float hmc1021_gauss);


#endif /* INC_PACKET_H_ */
