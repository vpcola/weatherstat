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
#include "soc/rtc_periph.h"
#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

// Components
#include "sht21x.h"
#include "stc3100.h"
#include "CayenneLPP.h"
#include "dustsensor_parser.h"

#include "TheThingsNetwork.h"

typedef enum
{
    EV_DUST_DATA_UPDATE,
    EV_LORA_MSG_RECV
} main_task_event_t;

typedef struct 
{
    main_task_event_t event;
    dustsensor_t dust_data;
} main_task_message_t;


// NOTE:
// The LoRaWAN frequency and the radio chip must be configured by running 'make menuconfig'.
// Go to Components / The Things Network, select the appropriate values and save.
const char *devEui = "00BD52FA56034180";
const char *appEui = "70B3D57ED002B79C";
const char *appKey = "B0A5E5F30925E5E425C9C514E80AC2CC";

// Pins and other resources
#define TTN_SPI_HOST      VSPI_HOST
#define TTN_SPI_DMA_CHAN  1
#define TTN_PIN_SPI_SCLK  GPIO_NUM_18
#define TTN_PIN_SPI_MOSI  GPIO_NUM_23
#define TTN_PIN_SPI_MISO  GPIO_NUM_19
#define TTN_PIN_NSS       GPIO_NUM_5
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       GPIO_NUM_25
#define TTN_PIN_DIO0      GPIO_NUM_34
#define TTN_PIN_DIO1      GPIO_NUM_35

#define BOARD_5V_ENA	  	GPIO_NUM_2 		// Must be an RTC gpio
#define REGULATOR_ON	  	1
#define REGULATOR_OFF	  	0

#define BOARD_I2C_PORT	  I2C_NUM_0
#define BOARD_I2C_SDA	  	GPIO_NUM_21
#define BOARD_I2C_SCL	  	GPIO_NUM_22
#define BOARD_I2C_SPEED	  100000

#define BOARD_LED					GPIO_NUM_12		// Must be an RTC gpio

#define DEEP_SLEEP_SECONDS	(60 * 5) // Wakeup every 10 minutes

#define MAX_MESSAGE_LEN	  42

#define DUSTSENSOR_UART_PORT UART_NUM_1
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


void sendMessages(void* pvParameter)
{
		main_task_message_t msg;
    float temp, humid, batvol, batcur, batrmc, battemp;
    bool hasDustData = false;

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

		// Wait until we have dust data
		while(!hasDustData)
		{
			xQueueReceive(main_task_queue, &(msg), portMAX_DELAY);
			switch(msg.event)
			{
				case EV_DUST_DATA_UPDATE:
						ESP_LOGI(TAG, "Receive dust sensor update!");
						// Add dust data to lpp, only send standard measurements
						lpp.addDigitalOutput(3, (uint8_t) msg.dust_data.pm1);
						lpp.addDigitalOutput(4, (uint8_t) msg.dust_data.pm25);
						lpp.addDigitalOutput(5, (uint8_t) msg.dust_data.pm10);
        		ESP_LOGI(TAG, "Concentration Unit (Standard): PM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum", 
                msg.dust_data.pm1,
                msg.dust_data.pm25,
                msg.dust_data.pm10);
        		ESP_LOGI(TAG, "Concentration Unit (Environmental): PM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum", 
        		    msg.dust_data.pm1_atmospheric,
                msg.dust_data.pm25_atmospheric,
                msg.dust_data.pm10_atmospheric);
                						
						hasDustData = true;
						break;
				default:
					ESP_LOGE(TAG, "Unknown event type!");
			}
		}				
				
    printf("Sending message...\n");
    TTNResponseCode res = ttn.transmitMessage(lpp.getBuffer(), lpp.getSize());
    printf(res == kTTNSuccessfulTransmission ? "Message sent.\n" : "Transmission failed.\n");
    	
    // Signal the main routine that we exited
    xSemaphoreGive(shutdown_sem);
		// Remove this task
		vTaskDelete(NULL);
}

void messageReceived(const uint8_t * message, size_t length, port_t port)
{
	printf("Message of %d bytes received on port %d:", length, port);
	for(int i = 0; i< length; i++)
		printf(" %02x", message[1]);
	printf("\n");
}

esp_err_t i2c_master_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed)
{
	esp_err_t err;
	
	i2c_config_t config;
	config.mode = I2C_MODE_MASTER;
	config.sda_io_num = sda;
	config.sda_pullup_en = GPIO_PULLUP_ENABLE;
	config.scl_io_num = scl;
	config.scl_pullup_en = GPIO_PULLUP_ENABLE;
	config.master.clk_speed = BOARD_I2C_SPEED;

	i2c_param_config(port, &config);
	err =  i2c_driver_install(port, config.mode, 0, 0, 0);
	
	ESP_LOGI(TAG, "I2C Master initialized with %d", err );
	
	return err;
}

static void i2c_master_deinit(i2c_port_t port, gpio_num_t sda, gpio_num_t scl)
{
    if (i2c_driver_delete(port) != ESP_OK)
        ESP_LOGE(TAG, "Failed to uninstall I2C driver!");
        
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
  
    ESP_LOGI(TAG, "I2C(%d) de-initialized", (int) port);
}

static esp_err_t spi_bus_init(void)
{
    esp_err_t err;

    spi_bus_config_t spi_bus_config;
    spi_bus_config.miso_io_num = TTN_PIN_SPI_MISO;
    spi_bus_config.mosi_io_num = TTN_PIN_SPI_MOSI;
    spi_bus_config.sclk_io_num = TTN_PIN_SPI_SCLK;
    spi_bus_config.quadwp_io_num = -1;
    spi_bus_config.quadhd_io_num = -1;
    spi_bus_config.max_transfer_sz = 0;
    err = spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN);
    ESP_ERROR_CHECK(err);

	  ESP_LOGI(TAG, "SPI(%d) Initialized", (int) TTN_SPI_HOST);
    return err;
}

static void spi_bus_deinit(void)
{
    // free the spi bus
    spi_bus_free(TTN_SPI_HOST);
   
    ESP_LOGI(TAG, "SPI(%d) de-initalized", (int) TTN_SPI_HOST);
}


static void dustsensor_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    dustsensor_t *sensor = NULL;
		main_task_message_t main_message;      
		
    switch (event_id) {
    case SENSOR_UPDATE:
        sensor = (dustsensor_t *)event_data;
        // Handle the data from the sensor here
        //ESP_LOGI(TAG, "Concentration Unit (Standard): PM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum", 
        //        sensor->pm1,
        //        sensor->pm25,
        //       sensor->pm10);
        //ESP_LOGI(TAG, "Concentration Unit (Environmental): PM1.0 = %d ug/cum, PM2.5 = %d ug/cum, PM10 = %d ug/cum", 
        //        sensor->pm1_atmospheric,
        //        sensor->pm25_atmospheric,
        //        sensor->pm10_atmospheric);
                
				main_message.event = EV_DUST_DATA_UPDATE;
				main_message.dust_data.pm1 = sensor->pm1;
				main_message.dust_data.pm25 = sensor->pm25;
				main_message.dust_data.pm10 = sensor->pm10;
				main_message.dust_data.pm1_atmospheric = sensor->pm1_atmospheric;
				main_message.dust_data.pm25_atmospheric = sensor->pm25_atmospheric;
				main_message.dust_data.pm10_atmospheric = sensor->pm10_atmospheric;
				// Send to queue
				xQueueSend(main_task_queue, (void *) &main_message, (TickType_t) 0);
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
    
    ESP_LOGI(TAG, "Dust Sensor Initialized");
}

static void dustsensor_deinit(void)
{
   // If the sleep enable pin is attached, we inhibit the 
   // sleep pin here.
   if (dustsensor_parser_deinit( dustsensor_hdl ) != ESP_OK)
       ESP_LOGE(TAG, "Dustsensor de-initialization error!\r\n");

   // Reset pins used by the dust sensor
   //if (rtc_gpio_is_valid_gpio(DUSTSENSOR_UART_RX))
   //	   rtc_gpio_isolate(DUSTSENSOR_UART_RX);
   
   ESP_LOGI(TAG, "Dust Sensor de-initialize");
}

static void boardled_init()
{
	
	  // Initialize the board led
    rtc_gpio_init(BOARD_LED);
    // Set the GPIO as a push/pull output 
    rtc_gpio_set_direction(BOARD_LED, RTC_GPIO_MODE_OUTPUT_ONLY);			  
    // Disable pullup/pulldown
    rtc_gpio_pulldown_dis(BOARD_LED);
    rtc_gpio_pullup_dis(BOARD_LED);
    
    // Turn the LED on
    rtc_gpio_set_level(BOARD_LED, 1);
    
    ESP_LOGI(TAG, "Board led turned on!");
}

static void boardled_shutdown()
{
    // Turn off the LED
    rtc_gpio_set_level(BOARD_LED, 0);
   
		ESP_LOGI(TAG, "Board led turned off!");
}

// Initialize the on board 5V regulator
// controlled by the BOARD_5V_ENA pin
static void board5V_init(void)
{
    // RTC gpio may have the HOLD function set
    // so clear it so that we can re-configure the pin
    rtc_gpio_hold_dis(BOARD_5V_ENA);

    // Initialize the BOARD_5V_ENA pin
    // as an RTC IO, disable pullup and pulldown
    rtc_gpio_init(BOARD_5V_ENA);
    rtc_gpio_set_direction(BOARD_5V_ENA, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(BOARD_5V_ENA);
    rtc_gpio_pullup_dis(BOARD_5V_ENA);
    // Enable the 5V regulator
    rtc_gpio_set_level(BOARD_5V_ENA, 1);
    
    ESP_LOGI(TAG, "Board 5V Regulator (%d) initialized", (int) BOARD_5V_ENA);
}



static void board5V_shutdown(void)
{
    rtc_gpio_set_level(BOARD_5V_ENA, 0); 
    // Prevent change of levels while on deep sleep
    rtc_gpio_hold_en(BOARD_5V_ENA);
    
    ESP_LOGI(TAG, "Board 5V regulator (%d) de-initialized", (int) BOARD_5V_ENA);
}


static void board_shutdown()
{
    // Shutdown RFM96W radio
    printf("Shutting down radio ...\n");
    ttn.shutdown();

    // Shutdown SPI port
    spi_bus_deinit();
    
    // Shutdown I2C port
    i2c_master_deinit(BOARD_I2C_PORT, BOARD_I2C_SDA, BOARD_I2C_SCL);
    
    // Shutdown the dust sensor
    dustsensor_deinit();

    // Power down the 5V regulator
    printf("Shutting down 5V regulator ...\n");
    board5V_shutdown();
}

extern "C" void app_main(void)
{
    esp_err_t err;
    
    // Disable deep sleep hold
    //gpio_deep_sleep_hold_dis();

    // Determine the actual sleep time
		gettimeofday(&now, NULL);
	
    // Initialize the GPIO ISR handler service
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    ESP_ERROR_CHECK(err);

    // Initialize the NVS (non-volatile storage) for saving and restoring the keys
    err = nvs_flash_init();
    ESP_ERROR_CHECK(err);

    // Initialize SPI bus
    //spi_bus_init();
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
    i2c_master_init(BOARD_I2C_PORT, BOARD_I2C_SDA, BOARD_I2C_SCL, BOARD_I2C_SPEED); 
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
		
		// Initialize and turn on LED
		boardled_init();    

    // Initialize GPIO pin for the 5V regulator
    board5V_init();

	  
    // Initialize the plantower sensor
    dustsensor_init();
    
    ESP_LOGI(TAG, "Creating main message queue\r\n");
    /* Create the main task message queue */
    main_task_queue = xQueueCreate(10, sizeof(main_task_message_t));
    if ( main_task_queue == 0)
    {
        ESP_LOGE(TAG, "Failed in creating main task queue!");
    }
    
    /* Create the shutdown semaphore */
    shutdown_sem = xSemaphoreCreateBinary();
    

    printf("Joining...\n");
    if (ttn.join())
    {
        printf("Re-Join/Joined sucessful!\n");
        xTaskCreate(sendMessages, "send_messages", 1024 * 4, (void* )0, 3, nullptr);
        
        // Wait untill all messages are sent
        xSemaphoreTake(shutdown_sem, portMAX_DELAY);
    }
    else
    {
        printf("Join failed. Goodbye\n");
    }
    
    // Delete the queue and semaphores
	  vQueueDelete(main_task_queue);
	  vSemaphoreDelete(shutdown_sem);
	  
		ESP_LOGI(TAG, "Shutdown initiated ...");
    // Shutdown peripherals and prepare for deep sleep
    board_shutdown();
    
    // Turn off LED
    boardled_shutdown();
    
    // Hold all gpios in deep sleep
    //gpio_deep_sleep_hold_en();

    // Set parameters to wake up from deep sleep, 
    // currently wakeup on timeout 
    ESP_LOGI(TAG, "Enabling timer wakeup in %ds", DEEP_SLEEP_SECONDS);
    const int wakeup_time = DEEP_SLEEP_SECONDS;
    esp_sleep_enable_timer_wakeup(wakeup_time * 1000000);
    
    gettimeofday(&sleep_enter_time, NULL);
    printf("Going deep sleep (%ds) ....\n", DEEP_SLEEP_SECONDS);
    esp_deep_sleep_start();        
}
