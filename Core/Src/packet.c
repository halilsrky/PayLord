/*
 * packet.c
 *
 *  Created on: May 7, 2025
 *      Author: Halil
 */
#include "packet.h"
#include <math.h>

unsigned char normal_paket[38];

unsigned char check_sum_hesapla_normal(int a){
    int check_sum = 0;
    for(int i = 4; i < a; i++){
        check_sum += normal_paket[i];
    }
    return (unsigned char) (check_sum % 256);
}


/*void addDataPacketNormal(BME_280_t* BME, bmi088_struct_t* BMI, gps_data_t* GPS, float hmc1021_gauss){
  normal_paket[0] = 0xFF; // Sabit
  normal_paket[1] = 0xFF; // Sabit
  normal_paket[2] = 0x54; // Sabit
  normal_paket[3] = 0x52; // Sabit

  normal_paket[4] = 0;   // Takim ID = 0
  normal_paket[5] = 0; // Sayac degeri = 0

  FLOAT32_UINT8_DONUSTURUCU irtifa_float32_uint8_donusturucu;
  irtifa_float32_uint8_donusturucu.sayi = (BME->altitude); // Irtifa degerinin atamasini yapiyoruz.
  normal_paket[6] = irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[7] = irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[8] = irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[9] = irtifa_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU roket_gps_irtifa_float32_uint8_donusturucu;
  roket_gps_irtifa_float32_uint8_donusturucu.sayi = (GPS->altitude); // Roket GPS Irtifa degerinin atamasini yapiyoruz.
  normal_paket[10] = roket_gps_irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[11] = roket_gps_irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[12] = roket_gps_irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[13] = roket_gps_irtifa_float32_uint8_donusturucu.array[3];

   // Roket Enlem
  FLOAT32_UINT8_DONUSTURUCU roket_enlem_float32_uint8_donusturucu;
  roket_enlem_float32_uint8_donusturucu.sayi = (GPS->latitude); // Roket enlem degerinin atamasini yapiyoruz.
  normal_paket[14] = roket_enlem_float32_uint8_donusturucu.array[0];
  normal_paket[15] = roket_enlem_float32_uint8_donusturucu.array[1];
  normal_paket[16] = roket_enlem_float32_uint8_donusturucu.array[2];
  normal_paket[17] = roket_enlem_float32_uint8_donusturucu.array[3];

  // Roket Boylam
  FLOAT32_UINT8_DONUSTURUCU roket_boylam_irtifa_float32_uint8_donusturucu;
  roket_boylam_irtifa_float32_uint8_donusturucu.sayi = (GPS->longitude); // Roket boylam degerinin atamasini yapiyoruz.
  normal_paket[18] = roket_boylam_irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[19] = roket_boylam_irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[20] = roket_boylam_irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[21] = roket_boylam_irtifa_float32_uint8_donusturucu.array[3];


  FLOAT32_UINT8_DONUSTURUCU aci_float32_uint8_donusturucu;
  aci_float32_uint8_donusturucu.sayi = (BMI->datas.theta); // Theta acisinin atamasini yapiyoruz.
  normal_paket[22] = aci_float32_uint8_donusturucu.array[0];
  normal_paket[23] = aci_float32_uint8_donusturucu.array[1];
  normal_paket[24] = aci_float32_uint8_donusturucu.array[2];
  normal_paket[25] = aci_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU volt_float32_uint8_donusturucu;
  volt_float32_uint8_donusturucu.sayi = (0); // Volt degerinin atamasini yapiyoruz.
  normal_paket[26] = volt_float32_uint8_donusturucu.array[0];
  normal_paket[27] = volt_float32_uint8_donusturucu.array[1];
  normal_paket[28] = volt_float32_uint8_donusturucu.array[2];
  normal_paket[29] = volt_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU akim_float32_uint8_donusturucu;
  akim_float32_uint8_donusturucu.sayi = (0); // Akim degerinin atamasini yapiyoruz.
  normal_paket[30] = akim_float32_uint8_donusturucu.array[0];
  normal_paket[31] = akim_float32_uint8_donusturucu.array[1];
  normal_paket[32] = akim_float32_uint8_donusturucu.array[2];
  normal_paket[33] = akim_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU sicaklik_float32_uint8_donusturucu;
  sicaklik_float32_uint8_donusturucu.sayi = (BME->temperature); // Sicaklik degerinin atamasini yapiyoruz.
  normal_paket[34] = sicaklik_float32_uint8_donusturucu.array[0];
  normal_paket[35] = sicaklik_float32_uint8_donusturucu.array[1];
  normal_paket[36] = sicaklik_float32_uint8_donusturucu.array[2];
  normal_paket[37] = sicaklik_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU basinc_float32_uint8_donusturucu;
  basinc_float32_uint8_donusturucu.sayi = (BME->pressure); // basinc degerinin atamasini yapiyoruz.
  normal_paket[38] = basinc_float32_uint8_donusturucu.array[0];
  normal_paket[39] = basinc_float32_uint8_donusturucu.array[1];
  normal_paket[40] = basinc_float32_uint8_donusturucu.array[2];
  normal_paket[41] = basinc_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU manyetik_alan_float32_uint8_donusturucu;
  manyetik_alan_float32_uint8_donusturucu.sayi = (hmc1021_gauss); // Manyetik alan degerinin atamasini yapiyoruz.
  normal_paket[42] = manyetik_alan_float32_uint8_donusturucu.array[0];
  normal_paket[43] = manyetik_alan_float32_uint8_donusturucu.array[1];
  normal_paket[44] = manyetik_alan_float32_uint8_donusturucu.array[2];
  normal_paket[45] = manyetik_alan_float32_uint8_donusturucu.array[3];

  //NEM
  normal_paket[46] = BME->humidity; // Nem degerinin atamasini yapiyoruz

  normal_paket[47] = flight_algorithm_get_durum_verisi(); // Durum bilgisi = Iki parasut de tetiklenmedi

  normal_paket[48] = check_sum_hesapla_normal(48); // Check_sum = check_sum_hesapla();
  normal_paket[49] = 0x0D; // Sabit
  normal_paket[50] = 0x0A;
}*/

void addDataPacketNormal(BME_280_t* BME, bmi088_struct_t* BMI, gps_data_t* GPS, float hmc1021_gauss){
  normal_paket[0] = 0xFF; // Sabit
  normal_paket[1] = 0xFF; // Sabit
  normal_paket[2] = 0x54; // Sabit
  normal_paket[3] = 0x52; // Sabit

  normal_paket[4] = 0;   // Takim ID = 0
  normal_paket[5] = 0; // Sayac degeri = 0

  FLOAT32_UINT8_DONUSTURUCU irtifa_float32_uint8_donusturucu;
  irtifa_float32_uint8_donusturucu.sayi = BMI->datas.acc_x; // Irtifa degerinin atamasini yapiyoruz.
  normal_paket[6] = irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[7] = irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[8] = irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[9] = irtifa_float32_uint8_donusturucu.array[3];


  FLOAT32_UINT8_DONUSTURUCU roket_gps_irtifa_float32_uint8_donusturucu;
  roket_gps_irtifa_float32_uint8_donusturucu.sayi = GPS->altitude; // Roket GPS Irtifa degerinin atamasini yapiyoruz.
  normal_paket[10] = roket_gps_irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[11] = roket_gps_irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[12] = roket_gps_irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[13] = roket_gps_irtifa_float32_uint8_donusturucu.array[3];

   // Roket Enlem
  FLOAT32_UINT8_DONUSTURUCU roket_enlem_float32_uint8_donusturucu;
  roket_enlem_float32_uint8_donusturucu.sayi = GPS->latitude; // Roket enlem degerinin atamasini yapiyoruz.
  normal_paket[14] = roket_enlem_float32_uint8_donusturucu.array[0];
  normal_paket[15] = roket_enlem_float32_uint8_donusturucu.array[1];
  normal_paket[16] = roket_enlem_float32_uint8_donusturucu.array[2];
  normal_paket[17] = roket_enlem_float32_uint8_donusturucu.array[3];

  // Roket Boylam
  FLOAT32_UINT8_DONUSTURUCU roket_boylam_irtifa_float32_uint8_donusturucu;
  roket_boylam_irtifa_float32_uint8_donusturucu.sayi = GPS->longitude; // Roket boylam degerinin atamasini yapiyoruz.
  normal_paket[18] = roket_boylam_irtifa_float32_uint8_donusturucu.array[0];
  normal_paket[19] = roket_boylam_irtifa_float32_uint8_donusturucu.array[1];
  normal_paket[20] = roket_boylam_irtifa_float32_uint8_donusturucu.array[2];
  normal_paket[21] = roket_boylam_irtifa_float32_uint8_donusturucu.array[3];


  FLOAT32_UINT8_DONUSTURUCU sicaklik_float32_uint8_donusturucu;
  sicaklik_float32_uint8_donusturucu.sayi = BME->temperature; // Sicaklik degerinin atamasini yapiyoruz.
  normal_paket[22] = sicaklik_float32_uint8_donusturucu.array[0];
  normal_paket[23] = sicaklik_float32_uint8_donusturucu.array[1];
  normal_paket[24] = sicaklik_float32_uint8_donusturucu.array[2];
  normal_paket[25] = sicaklik_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU basinc_float32_uint8_donusturucu;
  basinc_float32_uint8_donusturucu.sayi = BME->pressure; // Basinc degerinin atamasini yapiyoruz.
  normal_paket[26] = basinc_float32_uint8_donusturucu.array[0];
  normal_paket[27] = basinc_float32_uint8_donusturucu.array[1];
  normal_paket[28] = basinc_float32_uint8_donusturucu.array[2];
  normal_paket[29] = basinc_float32_uint8_donusturucu.array[3];

  FLOAT32_UINT8_DONUSTURUCU gorevVeri_float32_uint8_donusturucu;
  gorevVeri_float32_uint8_donusturucu.sayi = (hmc1021_gauss); // Basinc degerinin atamasini yapiyoruz.
  normal_paket[30] = gorevVeri_float32_uint8_donusturucu.array[0];
  normal_paket[31] = gorevVeri_float32_uint8_donusturucu.array[1];
  normal_paket[32] = gorevVeri_float32_uint8_donusturucu.array[2];
  normal_paket[33] = gorevVeri_float32_uint8_donusturucu.array[3];

  normal_paket[34] = BME->humidity; // Nem degerinin atamasini yapiyoruz

  normal_paket[35] = check_sum_hesapla_normal(35); // Check_sum = check_sum_hesapla();
  normal_paket[36] = 0x0D; // Sabit
  normal_paket[37] = 0x0A;
}

