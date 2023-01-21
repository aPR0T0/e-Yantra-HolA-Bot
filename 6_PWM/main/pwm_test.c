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
    // bit mask for the pins, each bit maps to a GPIO
    io_conf.pin_bit_mask = (1ULL<<19);
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

		gpio_set_level((gpio_num_t)18,1);
		for(int i = 0 ; i < stepPerRevolution ; i++)
		{
			gpio_set_level((gpio_num_t)19, 1);
			ets_delay_us(1000);
			gpio_set_level((gpio_num_t)19, 0);
			ets_delay_us(1000);
		}

		vTaskDelay(1000 / portTICK_PERIOD_MS);

		gpio_set_level((gpio_num_t)18,0);
		for(int i = 0 ; i < stepPerRevolution ; i++)
		{
			gpio_set_level((gpio_num_t)19, 1);
			ets_delay_us(1000);
			gpio_set_level((gpio_num_t)19, 0);
			ets_delay_us(1000);
		}

		vTaskDelay(1000 / portTICK_PERIOD_MS);
		
	}
	
	// Remove the task from the RTOS kernel management
	vTaskDelete(NULL);
}

void app_main()
{
	// xTaskCreate -> Create a new task and add it to the list of tasks that are ready to run
	xTaskCreate(&stepper_task, "stepper task", 4096, NULL, 1, NULL);
}