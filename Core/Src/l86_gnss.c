/*
 * l86_gnss.c
 *
 *  Created on: Jul 11, 2025
 *      Author: bilal
 */

#include "l86_gnss.h"

UART_HandleTypeDef *huart_gnss;
char gnss_rx_buffer[BUFFER_SIZE * 2];
char gps_buffer[BUFFER_SIZE];

// Add buffer synchronization
static volatile uint8_t buffer_ready_flag = 0;
static volatile uint8_t active_buffer = 0; // 0 = first half, 1 = second half

static char *gps_GNRMC_start_point = NULL;
static char *gps_GPGGA_start_point = NULL;
static uint8_t is_data_valid;
static char *current_char;
static char current_data[DATA_SIZE];
static uint8_t counter;
static char msg[MSG_SIZE];
static float non_formatted_latitude;
static float non_formatted_longitude;
static float non_formatted_time;
static uint32_t non_formatted_date;

static void set_baud_rate(L86_GNSS_BAUD_RATE baud_rate);
static uint8_t calculate_checksum(const char *data);
static void process_data(char *rx_buffer, uint16_t buffer_size);
static void get_GNRMC_data(gps_data_t *gps_data_);
static void get_GPGGA_data(gps_data_t *gps_data_);
static void format_data(gps_data_t *gps_data_);

void L86_GNSS_Init(UART_HandleTypeDef *huart_gnss_)
{
	huart_gnss = huart_gnss_;
	HAL_UART_Receive_DMA(huart_gnss, (uint8_t *)gnss_rx_buffer, BUFFER_SIZE * 2);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == huart_gnss)
	{
		// Disable interrupts briefly to prevent race condition
		__disable_irq();
		process_data(gnss_rx_buffer, BUFFER_SIZE);
		active_buffer = 0;
		buffer_ready_flag = 1;
		__enable_irq();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == huart_gnss)
	{
		// Disable interrupts briefly to prevent race condition
		__disable_irq();
		process_data(&gnss_rx_buffer[BUFFER_SIZE], BUFFER_SIZE);
		active_buffer = 1;
		buffer_ready_flag = 1;
		__enable_irq();
	}
}

void L86_GNSS_Update(gps_data_t *gps_data_)
{
	// Only process if new data is available
	if (buffer_ready_flag)
	{
		__disable_irq();
		buffer_ready_flag = 0;
		__enable_irq();
		
		get_GNRMC_data(gps_data_);
		get_GPGGA_data(gps_data_);
		format_data(gps_data_);
	}
}

void L86_GNSS_Print_Info(gps_data_t *gps_data_, UART_HandleTypeDef *huart_Seri_Port)
{
	memset(msg, 0, MSG_SIZE);

	if(gps_data_->is_valid == VALID)
	{
		snprintf(msg, MSG_SIZE, "Latitude: %f %c, Longitude: %f %c, Time: %u.%u.%u, Date: %u/%u/%u\r\n"
					"Speed: %f, Course: %f, Satellites in use: %u, HDOP: %f, Altitude: %f, Geoid height: %f, Orthometric height: %f\r\n",
					gps_data_->latitude, gps_data_->N_S, gps_data_->longitude, gps_data_->E_W,
					gps_data_->time[0], gps_data_->time[1], gps_data_->time[2], gps_data_->date[0], gps_data_->date[1], gps_data_->date[2],
					gps_data_->speed_over_ground, gps_data_->course_over_ground, gps_data_->satellites_in_use, gps_data_->HDOP,
					gps_data_->altitude, gps_data_->geoid_height, gps_data_->orthometric_height);
	}
	else
	{
		memcpy(msg, "Invalid Data!\r\n", 15);
	}

	HAL_UART_Transmit(huart_Seri_Port, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

static void process_data(char *rx_buffer, uint16_t buffer_size)
{
	// Ensure null termination for string functions
	memcpy(gps_buffer, rx_buffer, buffer_size);
	
	// Add null termination at the end to prevent buffer overflow
	if (buffer_size < BUFFER_SIZE) {
		gps_buffer[buffer_size] = '\0';
	} else {
		gps_buffer[BUFFER_SIZE - 1] = '\0';
	}
}

static void get_GNRMC_data(gps_data_t *gps_data_)
{
	gps_GNRMC_start_point = strstr(gps_buffer, "GNRMC");

	if(gps_GNRMC_start_point != NULL && (gps_GNRMC_start_point + 17) < (gps_buffer + BUFFER_SIZE))
	{
		if (*(gps_GNRMC_start_point + 17) == VALID)
		{
			is_data_valid = 1;
		}
		else
		{
			is_data_valid = 0;
			gps_data_->is_valid = INVALID;
		}
	}
	else
	{
		is_data_valid = 0;
		gps_data_->is_valid = INVALID;
	}

	if(is_data_valid == 1)
	{
		memset(current_data, 0, DATA_SIZE);
		counter = 0;
		current_char = gps_GNRMC_start_point;
		
		// Improved bounds checking
		while(*current_char != '*' && counter < (DATA_SIZE - 1) && 
			  current_char < (gps_buffer + BUFFER_SIZE))
		{
			current_data[counter] = *current_char;
			counter++;
			current_char++;
		}
		current_data[counter] = '\0'; // Ensure null termination

		sscanf(current_data, "GNRMC,%f,%c,%f,%c,%f,%c,%f,%f,%lu,,,%c",
				&gps_data_->non_fixed_time, &gps_data_->is_valid, &gps_data_->non_fixed_latitude, &gps_data_->N_S,
				&gps_data_->non_fixed_longitude, &gps_data_->E_W, &gps_data_->speed_over_ground, &gps_data_->course_over_ground,
				&gps_data_->non_fixed_date ,&gps_data_->positioning_mode);
	}

	gps_GNRMC_start_point = NULL;
}

static void get_GPGGA_data(gps_data_t *gps_data_)
{
	gps_GPGGA_start_point = strstr(gps_buffer, "GPGGA");

	if(gps_GPGGA_start_point != NULL && gps_GPGGA_start_point < (gps_buffer + BUFFER_SIZE))
	{
		if(gps_data_->is_valid == VALID)
		{
			memset(current_data, 0, DATA_SIZE);
			counter = 0;
			current_char = gps_GPGGA_start_point;
			
			// Improved bounds checking
			while(*(current_char) != '*' && counter < (DATA_SIZE - 1) && 
				  current_char < (gps_buffer + BUFFER_SIZE))
			{
				current_data[counter] = *current_char;
				counter++;
				current_char++;
			}
			current_data[counter] = '\0'; // Ensure null termination

			sscanf(current_data, "GPGGA,%f,%f,%c,%f,%c,%u,%u,%f,%f,M,%f,M,,",
					&gps_data_->non_fixed_time, &gps_data_->non_fixed_latitude, &gps_data_->N_S,
					&gps_data_->non_fixed_longitude, &gps_data_->E_W, &gps_data_->fix_status, &gps_data_->satellites_in_use,
					&gps_data_->HDOP, &gps_data_->altitude, &gps_data_->geoid_height);
		}

		gps_GPGGA_start_point = NULL;
	}
}

static void format_data(gps_data_t *gps_data_)
{
	if(gps_data_->is_valid == VALID)
	{
		// format latitude
		non_formatted_latitude = gps_data_->non_fixed_latitude;
		gps_data_->latitude = (float)floor(non_formatted_latitude / 100);
		non_formatted_latitude -= gps_data_->latitude * 100;
		non_formatted_latitude /= 60;
		gps_data_->latitude += non_formatted_latitude;

		// format langitude
		non_formatted_longitude = gps_data_->non_fixed_longitude;
		gps_data_->longitude = (float)floor(non_formatted_longitude / 100);
		non_formatted_longitude -= gps_data_->longitude * 100;
		non_formatted_longitude /= 60;
		gps_data_->longitude += non_formatted_longitude;

		// format time
		non_formatted_time = gps_data_->non_fixed_time;
		gps_data_->time[0] = (uint8_t)floor(non_formatted_time / 10000);
		non_formatted_time -= gps_data_->time[0] * 10000;
		gps_data_->time[0] += 3;
		if(gps_data_->time[0] >= 24)
		{
			gps_data_->time[0] -= 24;
		}
		gps_data_->time[1] = (uint8_t)floor(non_formatted_time / 100);
		non_formatted_time-= gps_data_->time[1] * 100;
		gps_data_->time[2] = (uint8_t)floor(non_formatted_time);

		// format date
		non_formatted_date = gps_data_->non_fixed_date;
		gps_data_->date[0] = (uint8_t)(non_formatted_date / 10000);
		non_formatted_date -= gps_data_->date[0] * 10000;
		gps_data_->date[1] = (uint8_t)(non_formatted_date / 100);
		non_formatted_date -= gps_data_->date[1] * 100;
		gps_data_->date[2] = (uint8_t)(non_formatted_date);

		// format height
		gps_data_->orthometric_height = gps_data_->altitude - gps_data_->geoid_height;
	}
}
