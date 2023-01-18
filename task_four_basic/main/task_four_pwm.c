//Components
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

//The main task to balance the robot
void stepper_task(void *arg)
{
	/**
	 * euler_angles are the complementary pitch and roll angles obtained from mpu6050
	 * mpu_offsets are the initial accelerometer angles at rest position
	*/
	float euler_angle[2], mpu_offset[2] = {0.0f, 0.0f};

	float pitch_angle, pitch_error;

	/**
	 * 1. motor_cmd : Stores theoritically calculated correction values obtained with PID.
	 * 2. motor_pwm : Variable storing bounded data obtained from motor_cmd which will be used for
	                  giving actual correction velocity to the motors
	*/
	float motor_cmd, motor_pwm = 0.0f;

	// Pitch angle where you want to go - pitch_cmd, setpoint and mpu_offsets are linked to one another
	float pitch_cmd = 0.0f;
#ifdef CONFIG_ENABLE_OLED
    // Initialising the OLED
    ESP_ERROR_CHECK(init_oled());
	vTaskDelay(100);

    // Clearing the screen
    lv_obj_clean(lv_scr_act());
#endif

	// Ensure successful initialisation of MPU-6050
	if (enable_mpu6050() == ESP_OK)
	{
		// Function to enable Motor driver A in Normal Mode
		enable_motor_driver(a, NORMAL_MODE);
		while (1)
		{
			/**
			 * read_mpu6050(euler_angle, mpu_offset) : Checking for successful calculation of complementary pitch 
			 *											and roll angles based on intial accelerometer angle
			*/
			// Ensure required values are obtained from mpu6050
			if (read_mpu6050(euler_angle, mpu_offset) == ESP_OK)
			{
				// To read PID setpoint from tuning_http_server
				pitch_cmd = read_pid_const().setpoint;
				pitch_angle = euler_angle[1];
				pitch_error = pitch_cmd - pitch_angle;

				calculate_motor_command(pitch_error, &motor_cmd);

				//bound PWM values between max and min
				motor_pwm = bound((motor_cmd), MIN_PWM, MAX_PWM);

				// Bot tilts upwards
				if (pitch_error > 1)
				{
					// setting motor A0 with definite speed(duty cycle of motor driver PWM) in Backward direction
					set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, motor_pwm);
					// setting motor A1 with definite speed(duty cycle of motor driver PWM) in Backward direction
					set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, motor_pwm);
				}

				// Bot tilts downwards
				else if (pitch_error < -1)
				{
					// setting motor A0 with definite speed(duty cycle of motor driver PWM) in Forward direction
					set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, motor_pwm);
					// setting motor A1 with definite speed(duty cycle of motor driver PWM) in Forward direction
					set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, motor_pwm);
				}

				// Bot remains in desired region for vertical balance
				else
				{
					// stopping motor A0
					set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
					// stopping motor A1
					set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
				}

				//ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
				ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f :: Setpoint: %0.2f :: Roll: %0.2f | Pitch: %0.2f | PitchError: %0.2f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd, read_pid_const().setpoint, euler_angle[0], euler_angle[1], pitch_error);
				// ESP_LOGI("debug", "Pitch: %0.2f", pitch_angle);
#ifdef CONFIG_ENABLE_OLED
				// Diplaying kp, ki, kd values on OLED
				if (read_pid_const().val_changed)
				{
					display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
					reset_val_changed_pid_const();
				}
#endif				
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
		}
	}

	// Remove the task from the RTOS kernel management
	vTaskDelete(NULL);
}

void app_main()
{
	// xTaskCreate -> Create a new task and add it to the list of tasks that are ready to run
	xTaskCreate(&stepper_task, "stepper task", 4096, NULL, 1, NULL);
}