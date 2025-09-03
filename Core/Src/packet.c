/*
 * packet.c
 *
 *  Created on: May 7, 2025
 *      Author: Halil
 */
#include "packet.h"
#include <math.h>

unsigned char normal_paket[50];
unsigned char sd_paket[64];


unsigned char check_sum_hesapla_normal(int a){
    int check_sum = 0;
    for(int i = 0; i < a; i++){
        check_sum += normal_paket[i];
    }
    return (unsigned char) (check_sum % 256);
}

void addDataPacketNormal(BME_280_t* BME, bmi088_struct_t* BMI, sensor_fusion_t* sensor, gps_data_t* GPS, float hmc1021_gauss, float voltage, float current){
  normal_paket[0] = 0xFF; // Sabit

  FLOAT32_UINT8_DONUSTURUCU irtifa_float32_uint8_donusturucu;
  irtifa_float32_uint8_donusturucu.sayi = (BME->altitude); // Irtifa degerinin atamasini yapiyoruz.
  normal_paket[1] = irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[2] = irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[3] = irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[4] = irtifa_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU roket_gps_irtifa_float32_uint8_donusturucu;
  roket_gps_irtifa_float32_uint8_donusturucu.sayi = (GPS->altitude); // Roket GPS Irtifa degerinin atamasini yapiyoruz.
  normal_paket[5] = roket_gps_irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[6] = roket_gps_irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[7] = roket_gps_irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[8] = roket_gps_irtifa_float32_uint8_donusturucu.array[3];

   // Roket Enlem
  FLOAT32_UINT8_DONUSTURUCU roket_enlem_float32_uint8_donusturucu;
  roket_enlem_float32_uint8_donusturucu.sayi = (GPS->latitude); // Roket enlem degerinin atamasini yapiyoruz.
  normal_paket[9] = roket_enlem_float32_uint8_donusturucu.array[0];
  normal_paket[10] = roket_enlem_float32_uint8_donusturucu.array[1];
  normal_paket[11] = roket_enlem_float32_uint8_donusturucu.array[2];
  normal_paket[12] = roket_enlem_float32_uint8_donusturucu.array[3];

  // Roket Boylam
  FLOAT32_UINT8_DONUSTURUCU roket_boylam_irtifa_float32_uint8_donusturucu;
  roket_boylam_irtifa_float32_uint8_donusturucu.sayi = (GPS->longitude); // Roket boylam degerinin atamasini yapiyoruz.
  normal_paket[13] = roket_boylam_irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[14] = roket_boylam_irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[15] = roket_boylam_irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[16] = roket_boylam_irtifa_float32_uint8_donusturucu.array[3];


  FLOAT32_UINT8_DONUSTURUCU aci_float32_uint8_donusturucu;
  aci_float32_uint8_donusturucu.sayi = (BMI->datas.theta); // Theta acisinin atamasini yapiyoruz.
  normal_paket[17] = aci_float32_uint8_donusturucu.array[0];
  normal_paket[18] = aci_float32_uint8_donusturucu.array[1];
  normal_paket[19] = aci_float32_uint8_donusturucu.array[2];
  normal_paket[20] = aci_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU volt_float32_uint8_donusturucu;
  volt_float32_uint8_donusturucu.sayi = (voltage); // Volt degerinin atamasini yapiyoruz.
  normal_paket[21] = volt_float32_uint8_donusturucu.array[0];
  normal_paket[22] = volt_float32_uint8_donusturucu.array[1];
  normal_paket[23] = volt_float32_uint8_donusturucu.array[2];
  normal_paket[24] = volt_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU akim_float32_uint8_donusturucu;
  akim_float32_uint8_donusturucu.sayi = (current); // Akim degerinin atamasini yapiyoruz.
  normal_paket[25] = akim_float32_uint8_donusturucu.array[0];
  normal_paket[26] = akim_float32_uint8_donusturucu.array[1];
  normal_paket[27] = akim_float32_uint8_donusturucu.array[2];
  normal_paket[28] = akim_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU sicaklik_float32_uint8_donusturucu;
  sicaklik_float32_uint8_donusturucu.sayi = (BME->temperature); // Sicaklik degerinin atamasini yapiyoruz.
  normal_paket[29] = sicaklik_float32_uint8_donusturucu.array[0];
  normal_paket[30] = sicaklik_float32_uint8_donusturucu.array[1];
  normal_paket[31] = sicaklik_float32_uint8_donusturucu.array[2];
  normal_paket[32] = sicaklik_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU basinc_float32_uint8_donusturucu;
  basinc_float32_uint8_donusturucu.sayi = (BME->pressure); // basinc degerinin atamasini yapiyoruz.
  normal_paket[33] = basinc_float32_uint8_donusturucu.array[0];
  normal_paket[34] = basinc_float32_uint8_donusturucu.array[1];
  normal_paket[35] = basinc_float32_uint8_donusturucu.array[2];
  normal_paket[36] = basinc_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU manyetik_alan_float32_uint8_donusturucu;
  manyetik_alan_float32_uint8_donusturucu.sayi = (hmc1021_gauss); // Manyetik alan degerinin atamasini yapiyoruz.
  normal_paket[37] = manyetik_alan_float32_uint8_donusturucu.array[0];
  normal_paket[38] = manyetik_alan_float32_uint8_donusturucu.array[1];
  normal_paket[39] = manyetik_alan_float32_uint8_donusturucu.array[2];
  normal_paket[40] = manyetik_alan_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU hiz_float32_uint8_donusturucu;
  hiz_float32_uint8_donusturucu.sayi = (sensor->velocity); // hiz degerinin atamasini yapiyoruz.
  normal_paket[41] = hiz_float32_uint8_donusturucu.array[0];
  normal_paket[42] = hiz_float32_uint8_donusturucu.array[1];
  normal_paket[43] = hiz_float32_uint8_donusturucu.array[2];
  normal_paket[44] = hiz_float32_uint8_donusturucu.array[3];

  //NEM
  normal_paket[45] = BME->humidity; // Nem degerinin atamasini yapiyoruz

  normal_paket[46] = flight_algorithm_get_durum_verisi();

  normal_paket[47] = check_sum_hesapla_normal(47); // Check_sum = check_sum_hesapla();
  normal_paket[48] = 0x0D; // Sabit
  normal_paket[49] = 0x0A;
}

void addDataPacketSD(BME_280_t* BME, bmi088_struct_t* BMI, sensor_fusion_t* sensor, gps_data_t* GPS, float hmc1021_gauss, float voltage, float current){

	  uint32_t zaman = HAL_GetTick();

	  sd_paket[0] = 0xFF; // Sabit

	  FLOAT32_UINT8_DONUSTURUCU irtifa_float32_uint8_donusturucu;
	  irtifa_float32_uint8_donusturucu.sayi = (BME->altitude); // Irtifa degerinin atamasini yapiyoruz.
	  sd_paket[1] = irtifa_float32_uint8_donusturucu.array[0];
	  sd_paket[2] = irtifa_float32_uint8_donusturucu.array[1];
	  sd_paket[3] = irtifa_float32_uint8_donusturucu.array[2];
	  sd_paket[4] = irtifa_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU roket_gps_irtifa_float32_uint8_donusturucu;
	  roket_gps_irtifa_float32_uint8_donusturucu.sayi = (GPS->altitude); // Roket GPS Irtifa degerinin atamasini yapiyoruz.
	  sd_paket[5] = roket_gps_irtifa_float32_uint8_donusturucu.array[0];
	  sd_paket[6] = roket_gps_irtifa_float32_uint8_donusturucu.array[1];
	  sd_paket[7] = roket_gps_irtifa_float32_uint8_donusturucu.array[2];
	  sd_paket[8] = roket_gps_irtifa_float32_uint8_donusturucu.array[3];

	   // Roket Enlem
	  FLOAT32_UINT8_DONUSTURUCU roket_enlem_float32_uint8_donusturucu;
	  roket_enlem_float32_uint8_donusturucu.sayi = (GPS->latitude); // Roket enlem degerinin atamasini yapiyoruz.
	  sd_paket[9] = roket_enlem_float32_uint8_donusturucu.array[0];
	  sd_paket[10] = roket_enlem_float32_uint8_donusturucu.array[1];
	  sd_paket[11] = roket_enlem_float32_uint8_donusturucu.array[2];
	  sd_paket[12] = roket_enlem_float32_uint8_donusturucu.array[3];

	  // Roket Boylam
	  FLOAT32_UINT8_DONUSTURUCU roket_boylam_irtifa_float32_uint8_donusturucu;
	  roket_boylam_irtifa_float32_uint8_donusturucu.sayi = (GPS->longitude); // Roket boylam degerinin atamasini yapiyoruz.
	  sd_paket[13] = roket_boylam_irtifa_float32_uint8_donusturucu.array[0];
	  sd_paket[14] = roket_boylam_irtifa_float32_uint8_donusturucu.array[1];
	  sd_paket[15] = roket_boylam_irtifa_float32_uint8_donusturucu.array[2];
	  sd_paket[16] = roket_boylam_irtifa_float32_uint8_donusturucu.array[3];


	  FLOAT32_UINT8_DONUSTURUCU jiroskop_x_float32_uint8_donusturucu;
	  jiroskop_x_float32_uint8_donusturucu.sayi = (BMI->datas.gyro_x); // Jiroskop X degerinin atamasini yapiyoruz.
	  sd_paket[17] = jiroskop_x_float32_uint8_donusturucu.array[0];
	  sd_paket[18] = jiroskop_x_float32_uint8_donusturucu.array[1];
	  sd_paket[19] = jiroskop_x_float32_uint8_donusturucu.array[2];
	  sd_paket[20] = jiroskop_x_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU jiroskop_y_float32_uint8_donusturucu;
	  jiroskop_y_float32_uint8_donusturucu.sayi = (BMI->datas.gyro_y); // Jiroskop Y degerinin atamasini yapiyoruz.
	  sd_paket[21] = jiroskop_y_float32_uint8_donusturucu.array[0];
	  sd_paket[22] = jiroskop_y_float32_uint8_donusturucu.array[1];
	  sd_paket[23] = jiroskop_y_float32_uint8_donusturucu.array[2];
	  sd_paket[24] = jiroskop_y_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU jiroskop_z_float32_uint8_donusturucu;
	  jiroskop_z_float32_uint8_donusturucu.sayi = (BMI->datas.gyro_z); // Jiroskop Z degerinin atamasini yapiyoruz.
	  sd_paket[25] = jiroskop_z_float32_uint8_donusturucu.array[0];
	  sd_paket[26] = jiroskop_z_float32_uint8_donusturucu.array[1];
	  sd_paket[27] = jiroskop_z_float32_uint8_donusturucu.array[2];
	  sd_paket[28] = jiroskop_z_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU ivme_x_float32_uint8_donusturucu;
	  ivme_x_float32_uint8_donusturucu.sayi = (BMI->datas.acc_x); // Ivme X degerinin atamasini yapiyoruz.
	  sd_paket[29] = ivme_x_float32_uint8_donusturucu.array[0];
	  sd_paket[30] = ivme_x_float32_uint8_donusturucu.array[1];
	  sd_paket[31] = ivme_x_float32_uint8_donusturucu.array[2];
	  sd_paket[32] = ivme_x_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU ivme_y_float32_uint8_donusturucu;
	  ivme_y_float32_uint8_donusturucu.sayi = (BMI->datas.acc_y); // Ivme Y degerinin atamasini yapiyoruz.
	  sd_paket[33] = ivme_y_float32_uint8_donusturucu.array[0];
	  sd_paket[34] = ivme_y_float32_uint8_donusturucu.array[1];
	  sd_paket[35] = ivme_y_float32_uint8_donusturucu.array[2];
	  sd_paket[36] = ivme_y_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU ivme_z_float32_uint8_donusturucu;
	  ivme_z_float32_uint8_donusturucu.sayi = (BMI->datas.acc_z); // Ivme Z degerinin atamasini yapiyoruz.
	  sd_paket[37] = ivme_z_float32_uint8_donusturucu.array[0];
	  sd_paket[38] = ivme_z_float32_uint8_donusturucu.array[1];
	  sd_paket[39] = ivme_z_float32_uint8_donusturucu.array[2];
	  sd_paket[40] = ivme_z_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU aci_float32_uint8_donusturucu;
	  aci_float32_uint8_donusturucu.sayi = (BMI->datas.theta); // Aci degerinin atamasini yapiyoruz.
	  sd_paket[41] = aci_float32_uint8_donusturucu.array[0];
	  sd_paket[42] = aci_float32_uint8_donusturucu.array[1];
	  sd_paket[43] = aci_float32_uint8_donusturucu.array[2];
	  sd_paket[44] = aci_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU manyetik_float32_uint8_donusturucu;
	  manyetik_float32_uint8_donusturucu.sayi = (hmc1021_gauss); // Sicaklik degerinin atamasini yapiyoruz.
	  sd_paket[45] = manyetik_float32_uint8_donusturucu.array[0];
	  sd_paket[46] = manyetik_float32_uint8_donusturucu.array[1];
	  sd_paket[47] = manyetik_float32_uint8_donusturucu.array[2];
	  sd_paket[48] = manyetik_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU hiz_float32_uint8_donusturucu;
	  hiz_float32_uint8_donusturucu.sayi = sensor->velocity; // HİZ degerinin atamasini yapiyoruz.
	  sd_paket[49] = hiz_float32_uint8_donusturucu.array[0];
	  sd_paket[50] = hiz_float32_uint8_donusturucu.array[1];
	  sd_paket[51] = hiz_float32_uint8_donusturucu.array[2];
	  sd_paket[52] = hiz_float32_uint8_donusturucu.array[3];

	  FLOAT32_UINT8_DONUSTURUCU zaman_float32_uint8_donusturucu;
	  zaman_float32_uint8_donusturucu.sayi = zaman; // zaman degerinin atamasini yapiyoruz.
	  sd_paket[53] = zaman_float32_uint8_donusturucu.array[0];
	  sd_paket[54] = zaman_float32_uint8_donusturucu.array[1];
	  sd_paket[55] = zaman_float32_uint8_donusturucu.array[2];
	  sd_paket[56] = zaman_float32_uint8_donusturucu.array[3];

	  //VOLT (voltage değerini union ile 2 byte olarak saklıyoruz)
	  UINT16_UINT8_DONUSTURUCU voltage_uint16_uint8_donusturucu;
	  voltage_uint16_uint8_donusturucu.sayi = (uint16_t)(voltage);
	  sd_paket[57] = voltage_uint16_uint8_donusturucu.array[0]; // High byte
	  sd_paket[58] = voltage_uint16_uint8_donusturucu.array[1]; // Low byte

	  //NEM
	  sd_paket[59] = BME->humidity; // Nem degerinin atamasini yapiyoruz
	  sd_paket[60] = flight_algorithm_get_durum_verisi(); // Durum bilgisi = Iki parasut de tetiklenmedi


	  sd_paket[61] = check_sum_hesapla_normal(61); // Check_sum = check_sum_hesapla();
	  sd_paket[62] = 0x0D; // Sabit
	  sd_paket[63] = 0x0A;
}
