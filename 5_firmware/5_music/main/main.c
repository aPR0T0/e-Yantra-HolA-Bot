#include "sra_board.h"
//#define debug
#include <time.h>
// C Headers
#include <stdio.h>
#include <math.h>

// Timer functions in FreeRTOS
#include "freertos/FreeRTOS.h"
#include <freertos/timers.h>
#include "driver/ledc.h"

const int stepsPerRevolution = 200;
static int taskCore = 0; // The core on which we need to run the task

const int kp_x = 8000;
const int kp_y = 1;
const int kp_z = 1;


const int angular_velocity = 5; // rad/sec -> vel_x, vel_y = s/R
const int k = 1; // Increase this only when u need a larger radius

#define highest_delay  500
#define lowest_delay  9

#define highest_speed  3.3
#define lowest_speed  0

// STEP and DIRECTION output pins for stepper motor driver.
static const gpio_num_t step_pin_m1 = GPIO_NUM_33;
static const gpio_num_t dire_pin_m1 = GPIO_NUM_32;

static const gpio_num_t step_pin_m2 = GPIO_NUM_14;
static const gpio_num_t dire_pin_m2 = GPIO_NUM_27;

static const gpio_num_t step_pin_m3 = GPIO_NUM_16;
static const gpio_num_t dire_pin_m3 = GPIO_NUM_17;

int32_t accel_limit = 0; // Hz. Speed change per update will not exceed this amount
int32_t speed_current = 0;  // Speed within acceleration limites sent to PWM.
// We will be varying the delay to control the speed of the bot

double x1, x2, x3; //These are the delay variables which will help us tweak delay for the speeds of the motors
int64_t before;
int count = 0;
int count_2 = 0; // Used to assist the step cycle

///////////////////////////////////////////////////////////////////////
/*
  Function   :    Timer
  Arguments  ;    none
  Returns    :    current time
*/
int64_t timer(){

  // Time function initialization
  if(count == 0){
    before = esp_timer_get_time();
    count++;
  }
  int64_t difference = esp_timer_get_time() - before;
  return difference;

}

///////////////////////////////////////////////////////////////////////
/*
  Function   :    differential timer
  Arguments  ;    none
  Returns    :    dTime
*/
int64_t d_timer(int64_t prev_time ){
  int64_t difference = esp_timer_get_time() - prev_time;
  return difference;
}

// ######################################################################################################################### //
// ######################################################################################################################### //


// ######################################################################################################################### //
// ##########################################      Main Function coming in      ############################################ //
// ######################################################################################################################### //

void stepper_task(void *arg){

  ledc_timer_config_t ledc_timer_m1 = {
        // Running Timer 0 in high speed mode. Not picky about which source
        // clock to use, so let it auto-select.
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .clk_cfg = LEDC_APB_CLK,

        .freq_hz = 10000,  
        .duty_resolution = LEDC_TIMER_12_BIT,
    };
  ledc_timer_config_t ledc_timer_m2 = {
        // Running Timer 0 in high speed mode. Not picky about which source
        // clock to use, so let it auto-select.
        .timer_num = LEDC_TIMER_1,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .clk_cfg = LEDC_APB_CLK,

        .freq_hz = 10000,  
        .duty_resolution = LEDC_TIMER_12_BIT,
    };
  ledc_timer_config_t ledc_timer_m3 = {
        // Running Timer 0 in high speed mode. Not picky about which source
        // clock to use, so let it auto-select.
        .timer_num = LEDC_TIMER_2,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .clk_cfg = LEDC_APB_CLK,

        .freq_hz = 10000,  
        .duty_resolution = LEDC_TIMER_12_BIT,
    };

  ledc_timer_config(&ledc_timer_m1);
  ledc_timer_config(&ledc_timer_m2);
  ledc_timer_config(&ledc_timer_m3);

  ledc_channel_config_t ledc_channel_one = {
    
    .timer_sel  = LEDC_TIMER_0,
    .speed_mode = LEDC_HIGH_SPEED_MODE,

    // Details for this output
    .channel    = LEDC_CHANNEL_0,
    .duty       = 0, // Start out stopped (0% duty cycle)
    .hpoint     = 0,
    .gpio_num   = step_pin_m1,
    };
  ledc_channel_config(&ledc_channel_one);

  ledc_channel_config_t ledc_channel_two = {
    
    .timer_sel  = LEDC_TIMER_1,
    .speed_mode = LEDC_HIGH_SPEED_MODE,

    // Details for this output
    .channel    = LEDC_CHANNEL_1,
    .duty       = 0, // Start out stopped (0% duty cycle)
    .hpoint     = 0,
    .gpio_num   = step_pin_m2,
    };
  ledc_channel_config(&ledc_channel_two);

  ledc_channel_config_t ledc_channel_three = {
    
    .timer_sel  = LEDC_TIMER_2,
    .speed_mode = LEDC_HIGH_SPEED_MODE,

    // Details for this output
    .channel    = LEDC_CHANNEL_2,
    .duty       = 0, // Start out stopped (0% duty cycle)
    .hpoint     = 0,
    .gpio_num   = step_pin_m3,
    };
  ledc_channel_config(&ledc_channel_three);

	gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 1,
        .pin_bit_mask = ((1ULL<<dire_pin_m1) | (1ULL<<dire_pin_m2) | (1ULL<<dire_pin_m3)),
    };
  
  ledc_set_duty(ledc_channel_one.speed_mode, ledc_channel_one.channel, 2048);
  ledc_update_duty(ledc_channel_one.speed_mode, ledc_channel_one.channel);
  ledc_set_duty(ledc_channel_two.speed_mode, ledc_channel_two.channel, 2048);
  ledc_update_duty(ledc_channel_two.speed_mode, ledc_channel_two.channel);
  ledc_set_duty(ledc_channel_three.speed_mode, ledc_channel_three.channel, 2048);
  ledc_update_duty(ledc_channel_three.speed_mode, ledc_channel_three.channel);
    

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
  // put your main code here, to run repeatedly:

	// float err_x, err_y, err_theta;

	// Now  to know the movements let me define the velocity vectors in x,y and angular velocity vector about z-axis
	// We will calculate the errors along them and then transform these vectors into individual velocities of the wheels

	int64_t time_z, time_t;
  
	while(1){
		time_z = timer();
    
    time_t = time_z / 1000000;                 
  // printf("x1 : %f x2 : %f and x3 : %f \n",x1, x2, x3);
  // printf("x1 : %lld x2 : %lld and x3 : %lld\n", (motor_one_curr_time- motor_one_prev_time), (motor_two_curr_time - motor_three_prev_time), (motor_three_curr_time - motor_three_prev_time));
    if(x1 >= 10){
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, abs(x1));
    }
    if(x2 >= 10){
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, abs(x2));
    }
    if(x3 >= 10){
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_2, abs(x3));
    }

		// Now we need to publish all the velocties for the individual wheel with the help of the parameters
		// speed_publisher( x1, x2, x3, vel_1, vel_2, vel_3);
	}
  
}
void app_main()
{
	// xTaskCreate -> Create a new task and add it to the list of tasks that are ready to run
	xTaskCreatePinnedToCore(&stepper_task, "stepper task", 8192, NULL, 1, NULL, taskCore); // Running the task on CORE0 only of the esp32
}