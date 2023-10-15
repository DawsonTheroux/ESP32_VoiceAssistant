#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_gpio.h"
#include "driver/i2s_std.h"
//#include "driver/i2s_pdm.h"

#define ESP_INTR_FLAG_DEFAULT 0 

// ----PIN DEFINITIONS----
// Define the butotn pin
#define BUTTON_PIN 39  

// Define the pins used for SPI LED control
#define LED_DATA_PIN 41
#define LED_CLOCK_PIN 40

// Define the pins used for I2S
#define PCM_DOUT_PIN 18
#define PCM_DIN_PIN 17
#define PCM_FS 16
#define PCM_CLK 15
// -----------------------
#define NUM_LEDS 3 // The number of LEDs in the chain.

int led_mode = 0;   // Can be 0(blue left),1(blue middle),2(blue right)
SemaphoreHandle_t button_semaphore; // The semaphore used for the button on the PiHAT

void get_LED_data(uint8_t* data){
  /* get_LED_data
   * 
   * Description:
   *  Gets the LED data for the three LEDs
   *  This function can be modified later to get other data.
   *
   * Parameters:
   *  data (uint8_t) - out 
  */
  int set_index;

  if(led_mode == 0){
    set_index = 0;
  }else if(led_mode == 1){
    set_index = 1;
  }else{
    set_index = 2;
  }

  int num_leds_set = 0;
  while (num_leds_set < NUM_LEDS) {
    data[(set_index * 4) + 0] = 0xEA; // Brightness
    data[(set_index * 4) + 1] = 0x00; // Blue colour value.
    data[(set_index * 4) + 2] = 0x00; // Green value
    data[(set_index * 4) + 3] = 0x00; // Red value
    
    if(num_leds_set == 0){
      data[(set_index * 4) + 1] = 0xFF; // Blue colour value.
    }else if(num_leds_set == 1){
      data[(set_index * 4) + 2] = 0xFF; // Green value
    }else if(num_leds_set == 2){
      data[(set_index * 4) + 3] = 0xFF; // Red value
    }
    num_leds_set += 1; //increment the number of LEDs set.
    
    // Change the set index to the next unset one.
    set_index += 1;
    if(set_index >= NUM_LEDS){
      set_index = 0;
    }
  }
}

void setup_spi_device(spi_device_handle_t *handle){
  /* setup_spi_device
   * 
   * Does the actual SPI device configuration for the LEDs
  */
  spi_bus_config_t buscfg={
    .miso_io_num = -1,
    .mosi_io_num = LED_DATA_PIN,
    .sclk_io_num = LED_CLOCK_PIN,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4096,
  };

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 1000000,
    // .clock_speed_hz = 1920,
    .mode = 0,
    .spics_io_num = -1,
    .queue_size = 1,
  };

  spi_bus_initialize(SPI3_HOST, &buscfg, 0);
  spi_bus_add_device(SPI3_HOST, &devcfg, handle);
}

void draw_LEDs(spi_device_handle_t handle){
  /* draw_LEDs
   *
   * This function sends the SPI transaction to the LEDs
   * The get_LED_data() function gets the actual LED state
   *
  */ 
  int packet_length = (NUM_LEDS * 4) + 4 + 4; // The length of the data to send to the LEDs
  uint8_t data[packet_length];                // The data to be sent to the LEDs

  // Set the start frame (32 0s)
  for(int i=0; i<4; i++){
   data[i] = 0x00;
  }

  // Set the LED data
  uint8_t led_data [NUM_LEDS * 4];
  get_LED_data(led_data);
  for(int i=0; i< NUM_LEDS * 4; i++){
    data[i + 4] = led_data[i];
  }

  // Set the end frame
  for(int i=0; i< 4; i++){
    data[(NUM_LEDS * 4) + i + 4] = 0xFF;
  }

  // Setup the transaction information.
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = packet_length * 8;
  t.tx_buffer = data;

  // Transmit the data and check for error.
  esp_err_t ret = spi_device_transmit(handle, &t);
  if(ret != ESP_OK){
    printf("AN ERROR HAS OCCURED WHEN TRANSMITTING THE LED DATA");
  }
}

void button_task(void* arg){
  /* button_taks 
   * 
   * This task waits until the button Semaphore is available
   * then updates the LED when it is available
  */
  spi_device_handle_t handle = (spi_device_handle_t)arg;

  for(;;){
    if(xSemaphoreTake(button_semaphore, portMAX_DELAY) == pdTRUE){
      printf("Button is pressed");
      led_mode += 1;
      if(led_mode >= NUM_LEDS){
        led_mode = 0;
      }
      draw_LEDs(handle);
    }
  }
}

void i2s_read_task(void *arg){
  printf("Starting read task\n");
  i2s_chan_handle_t i2s_chan = (i2s_chan_handle_t)arg;
  int num_samples = 10;
  int data_length = num_samples;
  char* data = malloc(data_length);
  for(;;){
    printf("Reading");
    size_t bytes_read;
    i2s_channel_read(i2s_chan, data, data_length, &bytes_read, portMAX_DELAY);
    printf("\n-- SAMPLE (bytes read: %d ) --\n",bytes_read);
    for(int i=0; i < (int)(bytes_read); i++){
      printf("Sample (%d): %d\n", i, data[i]);
    }
  }
}

void IRAM_ATTR button_isr_handler(void* arg){
  xSemaphoreGiveFromISR(button_semaphore,NULL);
}

void configure_i2s(i2s_chan_handle_t* i2s_chan){

  // 1. Determine the I2S channel and allocate the channel
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, i2s_chan));
  
  // 2. Configure the standard mode
  i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000), // Set sample rate to 44100Hz
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO), // Set slot configuration to 16-bit stereo
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PCM_CLK,
            .ws = PCM_FS,
            .dout = PCM_DOUT_PIN,
            .din = PCM_DIN_PIN,
        },
    };
    //ESP_ERROR_CHECK(i2s_channel_init_std(*i2s_chan, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*i2s_chan, &rx_std_cfg));
    i2s_channel_enable(*i2s_chan);

}


void app_main(void)
{
  // --------------------- I2S Setup ---------------------
  i2s_chan_handle_t i2s_chan;
  configure_i2s(&i2s_chan);
  xTaskCreate(i2s_read_task, "i2s_read_task", 8202, i2s_chan, 10, NULL);
  


  // // --------------------- SPI Setup ---------------------
  // spi_device_handle_t handle;
  // setup_spi_device(&handle);
  // // -----------------------------------------------------
  // // --------------------- Button Setup ---------------------
  // // Setup the button
  // button_semaphore = xSemaphoreCreateBinary();
  // esp_rom_gpio_pad_select_gpio(BUTTON_PIN);
  // gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
  // gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_POSEDGE);
  //
  // // Create the button task.
  // xTaskCreate(button_task, "button_task", 4096, handle, 10, NULL);
  // 
  // // Create button iterupt routine.
  // gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  // gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
  // // ---------------------------------------------------------
}
