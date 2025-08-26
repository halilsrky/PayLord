/*
 * data_logger.c
 *
 *  Created on: Jun 7, 2025
 *      Author: yahya
 */
#include "data_logger.h"
#include "fatfs.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

// Buffer için tanımlamalar
#define PACKET_SIZE 50
#define PACKETS_PER_SECTOR 10  // 10 paket = 500 byte (512'den küçük)
#define BUFFER_SIZE (PACKET_SIZE * PACKETS_PER_SECTOR)

static uint8_t packet_buffer[BUFFER_SIZE];
static uint16_t buffer_index = 0;

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; 	//Result after operations

//For file operation functions look at https://elm-chan.org/fsw/ff/00index_e.html

void data_logger_init()
{
	// SD kartı mount et
	fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK) {
		// Mount başarısız - hata işleme
		return;
	}
	
	// packets.bin dosyasını oluştur (eğer yoksa)
	fres = f_open(&fil, "packets.bin", FA_WRITE | FA_OPEN_ALWAYS);
	if (fres == FR_OK) {
		f_close(&fil);
	}
	
	// Buffer'ı sıfırla
	buffer_index = 0;
}

void log_normal_packet_data(unsigned char* packet_data)
{
	// Paketi buffer'a kopyala
	memcpy(&packet_buffer[buffer_index], packet_data, PACKET_SIZE);
	buffer_index += PACKET_SIZE;
	
	// Buffer doldu mu kontrol et
	if (buffer_index >= BUFFER_SIZE) {
		// Buffer'ı SD karta yaz
		flush_packet_buffer();
	}
}

void flush_packet_buffer(void)
{
	if (buffer_index > 0) {
		fres = f_open(&fil, "packets.bin", FA_WRITE | FA_OPEN_ALWAYS);
		if (fres == FR_OK) {
			f_lseek(&fil, f_size(&fil));
			unsigned int file_res = 0;
			
			// Buffer'daki tüm veriyi yaz
			f_write(&fil, packet_buffer, buffer_index, &file_res);
			f_close(&fil);
			
			// Buffer'ı sıfırla
			buffer_index = 0;
		}
	}
}

// Sistem kapanırken veya acil durumda buffer'ı boşalt
void force_flush_buffer(void)
{
	flush_packet_buffer();
}
