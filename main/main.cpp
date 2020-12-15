/*******************************************************************************
 * 
 * ttn-esp32 - The Things Network device library for ESP-IDF / SX127x
 * 
 * Copyright (c) 2018 Manuel Bleichenbacher
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 *
 * Sample program showing how to provision the keys via the console.
 *******************************************************************************/

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"

// Components
#include "sht21x.h"
#include "stc3100.h"
#include "CayenneLPP.h"
#include "dustsensor_parser.h"

#include "TheThingsNetwork.h"

// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.
const char *devEui = "00BD52FA56034180";
const char *appEui = "70B3D57ED002B79C";
const char *appKey = "B0A5E5F30925E5E425C9C514E80AC2CC";

// Pins and other resources
#define TTN_SPI_HOST      VSPI_HOST
#define TTN_SPI_DMA_CHAN  1
#define TTN_PIN_SPI_SCLK  18
#define TTN_PIN_SPI_MOSI  23
#define TTN_PIN_SPI_MISO  19
#define TTN_PIN_NSS       5
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       25
#define TTN_PIN_DIO0      34
#define TTN_PIN_DIO1      35

#define BOARD_5V_ENA	  GPIO_NUM_2 
#define REGULATOR_ON	  1
#define REGULATOR_OFF	  0

#define BOARD_I2C_PORT	  I2C_NUM_0
#define BOARD_I2C_SDA	  GPIO_NUM_21
#define BOARD_I2C_SCL	  GPIO_NUM_22
#define BOARD_I2C_SPEED	  100000

#define DEEP_SLEEP_SECONDS	60

#define MAX_MESSAGE_LEN	  42

#define DUSTSENSOR_UART_PORT UART_NUM_2
#define DUSTSENSOR_UART_RX	 GPIO_NUM_16

// Variable declarations
static TheThingsNetwork ttn;
static CayenneLPP lpp(MAX_MESSAGE_LEN); 

// Variables stored on slow memory, they're retained
// from deep sleep to deep sleep
static RTC_DATA_ATTR struct timeval sleep_enter_time;
static struct timeval now;
static RTC_DATA_ATTR int boot_count = 0;

static dustsensor_parser_handle_t dustsensor_hdl;
static QueueHandle_t main_task_queue;
static SemaphoreHandle_t shutdown_sem;

static const char *TAG = "main";

//const unsigned TX_INTERVAL = 30;


void sendMessages(void* pvParameter)
{
    float temp, humid, batvol, batcur, batrmc, battemp;

    //while (1) {
				// Read sensor data
				sht21_humidity(BOARD_I2C_PORT, &humid);
				printf("Relative Humidity: %f\n", humid);
				sht21_temperature(BOARD_I2C_PORT, &temp);
				printf("Temperature: %f\n", temp);
				stc3100_get_battery_voltage(BOARD_I2C_PORT, &batvol);
				printf("Battery Voltage : %.2f V\n", batvol);
				stc3100_get_battery_current(BOARD_I2C_PORT, &batcur);
				printf("Battery Current Draw: %.3f\n", batcur);
				stc3100_get_battery_rem_charge(BOARD_I2C_PORT, &batrmc);
				printf("Battery Remaining Charge: %.2f mAh\n", batrmc);
				stc3100_get_battery_temperature(BOARD_I2C_PORT, &battemp);
				printf("Battery Temperature : %.2f C\n", battemp);
				lpp.reset();
				lpp.addRelativeHumidity(0, humid);
				lpp.addTemperature(1, temp);
				lpp.addAnalogOutput(2, batvol);
				//lpp.addAnalogOutput(3, batcur);
				//lpp.addAnalogOutput(4, batrmc);
				//lpp.addAnalogOutput(6, battemp);

        printf("Sending message...\n");
        TTNResponseCode res = ttn.transmitMessage(lpp.getBuffer(), lpp.getSize());
        printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
	
        // vTaskDelay(TX_INTERVAL * pdMS_TO_TICKS(1000));
    //}
}

void messageReceived(const uint8_t * message, size_t length, port_t port)
{
	printf("Message of %d bytes received on port %d:", length, port);
	for(int i = 0; i< length; i++)
		printf(" %02x", message[1]);
	printf("\n");
}

esp_err_t i2c_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed)
{
	i2c_config_t config;
	config.mode = I2C_MODE_MASTER;
	config.sda_io_num = sda;
	config.sda_pullup_en = GPIO_PULLUP_ENABLE;
	config.scl_io_num = scl;
	config.scl_pullup_en = GPIO_PULLUP_ENABLE;
	config.master.clk_speed = BOARD_I2C_SPEED;

	i2c_param_config(port, &config);
	return i2c_driver_install(port, config.mode, 0, 0, 0);
}

static void dustsensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    dustsensor_t *sensor = NULL;
    switch (event_id) {
    case SENSOR_UPDATE:
        sensor = (dustsensor_t *)event_data;
        // Handle the data from the sensor here
        ESP_LOGI(TAG, "Concentration Unit (Standard):\r\n"
                "\tPM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum\r\n", 
                sensor->pm1,
                sensor->pm25,
                sensor->pm10);
        ESP_LOGI(TAG, "Concentration Unit (Environmental):\r\n"
                "\tPM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum\r\n", 
                sensor->pm1_atmospheric,
                sensor->pm25_atmospheric,
                sensor->pm10_atmospheric);

        break;
    case SENSOR_UNKNOWN:
        /* print unknown statements */
        ESP_LOGE(TAG, "Unknown statement:%s", (char *)event_data);
        break;
    default:
        break;
    }
}

static void dustsensor_init(void)
{
    /* NMEA parser configuration */
    dustsensor_parser_config_t config = DUSTSENSOR_PARSER_CONFIG_DEFAULT();
    config.uart.uart_port = (uart_port_t) DUSTSENSOR_UART_PORT;
    config.uart.rx_pin = DUSTSENSOR_UART_RX;
    /* init NMEA parser library */
    dustsensor_hdl = dustsensor_parser_init(&config);
    /* register event handler for NMEA parser library */
    dustsensor_parser_add_handler(dustsensor_hdl, dustsensor_event_handler, NULL);
}

static void dustsensor_deinit(void)
{
   // If the sleep enable pin is attached, we inhibit the 
   // sleep pin here.
   if (dustsensor_parser_deinit( dustsensor_hdl ) != ESP_OK)
       ESP_LOGE(TAG, "Dustsensor de-initialization error!\r\n");
}



extern "C" void app_main(void)
{
    esp_err_t err;

		// Determine the actual sleep time
		gettimeofday(&now, NULL);
	
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize GPIO pin for the 5V regulator
    gpio_pad_select_gpio( BOARD_5V_ENA);
    gpio_set_direction(BOARD_5V_ENA, GPIO_MODE_OUTPUT);
    gpio_set_level(BOARD_5V_ENA, 1);

    // Initialize SPI bus
    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

    // Initialize I2C bus
    i2c_init(BOARD_I2C_PORT, BOARD_I2C_SDA, BOARD_I2C_SCL, BOARD_I2C_SPEED); 
    stc3100_init(BOARD_I2C_PORT, boot_count);
 

    // Configure the SX127x pins
    ttn.configurePins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);
    
	  
    ttn.provision(devEui, appEui, appKey);
    ttn.onMessage(messageReceived);


		ESP_LOGI(TAG, "Boot Count = %d", boot_count);


		int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;
		switch (esp_sleep_get_wakeup_cause())
		{
			case ESP_SLEEP_WAKEUP_TIMER: 
									 {
										 ESP_LOGI(TAG,"Wake up from timer. Time spent in deep sleep: %d (ms)", sleep_time_ms);
										 break;
									 }
			default:
									 {
										 ESP_LOGI(TAG,"Wake up from other sources ....");
										 break;
									 }
		}    

	  boot_count++;
	  
    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Re-Join/Joined sucessful!\n");
        //xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
        sendMessages(NULL); 
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
    printf("Waiting for incoming messages (10s) ...\n");
    vTaskDelay(10 * pdMS_TO_TICKS(1000));
    
    // Power down lora radio
    printf("Shutting down radio ...\n");
    ttn.shutdown();
    
    // Power down the 5V regulator
    printf("Shutting down 5V regulator ...\n");
    gpio_set_level(BOARD_5V_ENA, 0);    
    
    // Set parameters to wake up from deep sleep, 
    // currently wakeup on timeout 
    ESP_LOGI(TAG, "Enabling timer wakeup in %ds", DEEP_SLEEP_SECONDS);
    const int wakeup_time = DEEP_SLEEP_SECONDS;
    esp_sleep_enable_timer_wakeup(wakeup_time * 1000000);
    
    gettimeofday(&sleep_enter_time, NULL);
    printf("Going deep sleep (%ds) ....\n", DEEP_SLEEP_SECONDS);
    esp_deep_sleep_start();
}
