#include "sra_board.h"
//#define debug

// C Headers
#include <stdio.h>
#include <math.h>

// static const gpio_num_t StepPin1 = 19;
// static const gpio_num_t dirPin1 = GPIO_NUM_18;
// static const gpio_num_t enPin1 = GPIO_NUM_5;

const int stepPerRevolution = 200;

//The main task to balance the robot
void stepper_task(void *arg)
{
	gpio_config_t io_conf;
    // bit mask for the pins, each bit maps to a GPIO//                                                                                                         motor 1               motor 2                 motor 3                                           
    io_conf.pin_bit_mask = ((1ULL<<GPIO_NUM_27) | (1ULL<<GPIO_NUM_14) | (1ULL<<GPIO_NUM_32) | (1ULL<<GPIO_NUM_33) | (1ULL<<GPIO_NUM_16) | (1ULL<<GPIO_NUM_17) | (1ULL<<GPIO_NUM_4) | (1ULL<<GPIO_NUM_12) | (1ULL<<GPIO_NUM_25));
    // set gpio mode to input
    io_conf.mode = GPIO_MODE_OUTPUT;
	// enable pull up resistors
    io_conf.pull_up_en = 0;
    // disable pull down resistors
    io_conf.pull_down_en = 1;
    // disable gpio interrupts
    io_conf.intr_type = GPIO_INTR_DISABLE;
    
	esp_err_t err = gpio_config(&io_conf);
	if (err == ESP_OK)
    {
        // ESP_LOGI(TAG_BAR_GRAPH, "enabled bar graph leds in mode: %d", enabled_bar_graph_flag);
		printf("ohk!");
    }
    else
    {
        // ESP_LOGE(TAG_BAR_GRAPH, "error: %s", esp_err_to_name(err));
        // enabled_bar_graph_flag = 0;
		printf("error!");
    }
	while (1)
	{
		gpio_set_level(GPIO_NUM_27,1);
		gpio_set_level(GPIO_NUM_32,0);
		gpio_set_level(GPIO_NUM_17,0);
		for(int t = 0;t<4; t++)
		{
			for(int i = 0 ; i < stepPerRevolution ; i++)
			{
				
				gpio_set_level(GPIO_NUM_14, 1);
				ets_delay_us(100);
				gpio_set_level(GPIO_NUM_33, 1);
				ets_delay_us(100);
				gpio_set_level(GPIO_NUM_16, 1);
				ets_delay_us(850);
				gpio_set_level(GPIO_NUM_14, 0);
				ets_delay_us(100);
				// gpio_set_level(GPIO_NUM_33, 0);
				// ets_delay_us(100);
				gpio_set_level(GPIO_NUM_16, 0);
				ets_delay_us(1000);
				
			}
		}
		vTaskDelay(100/portTICK_PERIOD_MS);
		gpio_set_level(GPIO_NUM_27,0);
		gpio_set_level(GPIO_NUM_32,0);
		gpio_set_level(GPIO_NUM_17,1);
		for(int t=0;t<4;t++){
			for(int i = 0 ; i < stepPerRevolution ; i++)
			{
				
				gpio_set_level(GPIO_NUM_14, 1);
				ets_delay_us(100);
				gpio_set_level(GPIO_NUM_33, 1);
				ets_delay_us(100);
				gpio_set_level(GPIO_NUM_16, 1);
				ets_delay_us(850);
				// gpio_set_level(GPIO_NUM_14, 0);
				// ets_delay_us(100);
				gpio_set_level(GPIO_NUM_33, 0);
				ets_delay_us(100);
				gpio_set_level(GPIO_NUM_16, 0);
				ets_delay_us(1000);
				
			}
		}
		vTaskDelay(100/portTICK_PERIOD_MS);
		gpio_set_level(GPIO_NUM_27,0);
		gpio_set_level(GPIO_NUM_32,1);
		gpio_set_level(GPIO_NUM_17,1);
		for(int t=0;t<4;t++){
			for(int i = 0 ; i < stepPerRevolution ; i++)
			{
				
				gpio_set_level(GPIO_NUM_14, 1);
				ets_delay_us(100);
				gpio_set_level(GPIO_NUM_33, 1);
				ets_delay_us(100);
				gpio_set_level(GPIO_NUM_16, 1);
				ets_delay_us(850);
				gpio_set_level(GPIO_NUM_14, 0);
				ets_delay_us(100);
				gpio_set_level(GPIO_NUM_33, 0);
				// ets_delay_us(100);
				// gpio_set_level(GPIO_NUM_16, 0);
				ets_delay_us(1000);
				
			}
		}
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}
void app_main()
	{
	// xTaskCreate -> Create a new task and add it to the list of tasks that are ready to run
	xTaskCreate(&stepper_task, "stepper task", 4096, NULL, 1, NULL);
}