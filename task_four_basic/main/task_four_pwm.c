3//Components
#include "sra_board.h"
//#define debug

// C Headers
#include <stdio.h>
#include <math.h>

static const gpio_num_t StepPin1 = GPIO_NUM_19;
static const gpio_num_t dirPin1 = GPIO_NUM_18;
static const gpio_num_t enPin1 = GPIO_NUM_5;   

static const gpio_num_t StepPin2 = GPIO_NUM_32;
static const gpio_num_t dirPin2 = GPIO_NUM_33;
static const gpio_num_t enPin2 = GPIO_NUM_25;   

static const gpio_num_t StepPin3 = GPIO_NUM_26;
static const gpio_num_t dirPin3 = GPIO_NUM_27;
static const gpio_num_t enPin3 = GPIO_NUM_14; 

const int stepPerRevolution = 200;

//The main task to balance the robot
void stepper_task(void *arg)
{
	while (1)
	{

		gpio_set_level(dirPin1, 1);
		for(int i = 0 ; i < stepPerRevolution ; i++)
		{
			gpio_set_level(StepPin1, 1);
			vTaskDelay(0.45 / portTICK_PERIOD_MS);
			gpio_set_level(StepPin1, 0);
			vTaskDelay(0.45 / portTICK_PERIOD_MS);
		}

		vTaskDelay(100 / portTICK_PERIOD_MS);
		for(int i = 0; i< 10 ; i++){
			gpio_set_level(StepPin1, 1);
			vTaskDelay(100 / portTICK_PERIOD_MS);
			gpio_set_level(enPin1, 1);
			vTaskDelay(100 / portTICK_PERIOD_MS);

			gpio_set_level(enPin1, 0);
		}

		//ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
		ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f :: Setpoint: %0.2f :: Roll: %0.2f | Pitch: %0.2f | PitchError: %0.2f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd, read_pid_const().setpoint, euler_angle[0], euler_angle[1], pitch_error);

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