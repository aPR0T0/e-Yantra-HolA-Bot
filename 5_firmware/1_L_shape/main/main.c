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


const float angular_velocity = 0.6; // rad/sec -> vel_x, vel_y = s/R
const float radius = 0.3; // Increase this only when u need a larger radius

#define highest_frequency  600
#define lowest_frequency  18

#define highest_speed  0.67
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
////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////
/*
  Function   :    modulus
  Arguments  ;    double -> number
  Working    :    Gets the modulus of the given number
  Returns    :    a number -> double
*/
double modulus(double z){

  if(z<0){
    z = -z;
  }
  return z; 

}
///////////////////////////////////////////////////////////////////////


// ##### This part is only for the inverse kinematics to get the solution of for the desired output at any given time ##### //

/*
  Function    :   determinantOfMatrix()
  Arguments   :   a 3x3 matrix with all the elements in it
  type        :   double[][]
  Returns     :   Determinant of the given 2d matrix 
*/

double determinantOfMatrix(double mat[3][3])
{
    double ans;
    ans =   mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
          - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
          + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
    return ans;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This function finds the solution of system of
// linear equations using cramer's rule

/*
  Function    :   findSolution()
  Arguments   :   a 3x4 matrix with all the elements in it
  type        :   double[][]
  Returns     :   a an array which represents a 3x1 matrix and all the elements in order
*/
double* findSolution(double coeff[3][4])
{     
    double *ans = malloc(3*sizeof(double));
    // Matrix d using coeff as given in cramer's rule
    double d[3][3] = {
        { coeff[0][0], coeff[0][1], coeff[0][2] },
        { coeff[1][0], coeff[1][1], coeff[1][2] },
        { coeff[2][0], coeff[2][1], coeff[2][2] },
    };
    // Matrix d1 using coeff as given in cramer's rule
    double d1[3][3] = {
        { coeff[0][3], coeff[0][1], coeff[0][2] },
        { coeff[1][3], coeff[1][1], coeff[1][2] },
        { coeff[2][3], coeff[2][1], coeff[2][2] },
    };
    // Matrix d2 using coeff as given in cramer's rule
    double d2[3][3] = {
        { coeff[0][0], coeff[0][3], coeff[0][2] },
        { coeff[1][0], coeff[1][3], coeff[1][2] },
        { coeff[2][0], coeff[2][3], coeff[2][2] },
    };
    // Matrix d3 using coeff as given in cramer's rule
    double d3[3][3] = {
        { coeff[0][0], coeff[0][1], coeff[0][3] },
        { coeff[1][0], coeff[1][1], coeff[1][3] },
        { coeff[2][0], coeff[2][1], coeff[2][3] },
    };

    // Calculating Determinant of Matrices d, d1, d2, d3
    double D  = determinantOfMatrix(d) ;
    double D1 = determinantOfMatrix(d1);
    double D2 = determinantOfMatrix(d2);
    double D3 = determinantOfMatrix(d3);

    // Case 1
    if (D != 0) {
        // Coeff have a unique solution. Apply Cramer's Rule
        double x = D1 / D;
        double y = D2 / D;
        double z = D3 / D; // calculating z using cramer's rule

        ans[0] = x;
        ans[1] = y;
        ans[2] = z;
    }
    // Case 2
    else {
      // this says your velocities are invalid
      for(int i= 0;i<3;i++){
          ans[i] = 0;
      }
    }
    
    return ans; // This will give the velocity in RPM
}

// ############################################################################################################## //
// ############################################################################################################## //

// ################### Speed Publisher for all the motors according to the delays provided ###################### //


/*
  Function    :   speed_publisher()
  Arguments   :   1. int x1 which are the delays
                  2. int x2 which are the delays
                  3. int x3 which are the delays
                  4. int vel_1 which is the velocity for the wheel 1
                  5. int vel_2 which is the velocity for the wheel 2
                  6. int vel_3 which is the velocity for the wheel 3
  Returns     :   Nothing - > void
*/
void speed_publisher(double x1, double x2, double x3, double vel_1, double vel_2, double vel_3){	// Publishes Speed according to the given body frame velocities

  // Now we need to transform the velocity in rpm to the delay in microSec

  if(vel_1 > 0){
    gpio_set_level(GPIO_NUM_32 , 0); // clockwise direction 
    if(vel_1 > highest_speed){
      vel_1 = highest_speed;
    }
  }
  else if( vel_1 <= 0){
    gpio_set_level(GPIO_NUM_32 , 1); // anticlockwise direction
    if(vel_1 < -highest_speed){
      vel_1 = -highest_speed;
    }
  }
  if(vel_2 > 0){
    gpio_set_level(GPIO_NUM_17 , 0);
    if(vel_2 > highest_speed){
      vel_2 = highest_speed;
    }
  }
  else if(vel_2 <= 0){
    gpio_set_level(GPIO_NUM_17 , 1);
    if(vel_2 < -highest_speed){
      vel_2 = -highest_speed;
    }
  }
  if(vel_3 > 0){
    gpio_set_level(GPIO_NUM_27 , 0);
    if(vel_3 > highest_speed){
      vel_3 = highest_speed;
    }
  }
  else if(vel_3 <= 0){
    gpio_set_level(GPIO_NUM_27 , 1);
    if(vel_3 < -highest_speed){
      vel_3 = -highest_speed;
    }
  }

  // Now, just mapping the velocities to the delays

  vel_1 = modulus(vel_1);   // Direction pins are already set 
  vel_2 = modulus(vel_2);   // Direction pins are already set 
  vel_3 = modulus(vel_3);   // Direction pins are already set 
  
  x1 = (vel_1/0.029) * 35 + 20; // 1 rad/sec = 35 hz
  x2 = (vel_2/0.029) * 35 + 20; // 1 rad/sec = 35 hz
  x3 = (vel_3/0.029) * 35 + 20; // 1 rad/sec = 35 hz

  if(x1 > highest_frequency){
    x1 = highest_frequency;
    
  }
  if(x2 > highest_frequency){
    x2 = highest_frequency;
  }
  if(x3 > highest_frequency){ 
    x3 = highest_frequency;
  }

  printf("x1 : %f x2 : %f and x3 : %f \n",x1, x2, x3);
  // printf("x1 : %lld x2 : %lld and x3 : %lld\n", (motor_one_curr_time- motor_one_prev_time), (motor_two_curr_time - motor_three_prev_time), (motor_three_curr_time - motor_three_prev_time));
  if(x1 >= 20){
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, abs(x1));
  }
  else{
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, 1);
  }
  if(x2 >= 20){
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, abs(x2));
  }
  else{
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, 1);
  }
  if(x3 >= 20){
    ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_2, abs(x3));
  }
  else{
    ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_2, 1);
  }
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
        .pin_bit_mask = ((1ULL<<dire_pin_m1) | (1ULL<<dire_pin_m2) | (1ULL<<dire_pin_m3) ),
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
	double vel_x, vel_y, vel_z, vel_1, vel_2, vel_3;
	double *velocities;       // The final Velocity matrix after the allocation for each wheels

	// Rotation matrix definition just for the next gen code
	double rot_z = 0;
	double theta;
	double time_z, time_t;

	double rotation_matrix[3][3] = {{     cos(rot_z)    ,    sin(rot_z)    , 0},\
                                  {    -sin(rot_z)    ,    cos(rot_z)    , 0},\
                                  {         0         ,        0         , 1}};
  
  
	while(1){
    time_t = 0;
      time_z = timer();
      time_t = time_z / 1000000; // As time was in microseconds
      if(time_t < 3){
        printf("loop1");
      // printf(" some time: %f \n",time_t);
      vel_x = 0.2;  // v = r*Ω*sin(Θ) 
      vel_y = 0;  // v = r*Ω*cos(Θ)
      vel_z = 0;                                // equation for the circle i.e. no rotation
      // printf(" vel_x : %f vel_y : %f  \n", vel_x, vel_y);

      double coefficients[3][4] =  {{     -1    ,       0.5        ,     0.5         , vel_x  },\
                                    {      0    , 	-sqrt(3)/2     ,   sqrt(3)/2     , vel_y  },\
                                    {     1.0   ,       1.0        ,     1.0         , vel_z  }};  // The allocation matrix along with a column of the desired velocities
      
      velocities = findSolution(coefficients);

      vel_1 = velocities[0];
      vel_2 = velocities[1];
      vel_3 = velocities[2];
      printf("vel_1: %f, vel_2: %f, and vel_3: %f \n", vel_1, vel_2, vel_3);
      free(velocities);

      // Now we need to publish all the velocties for the individual wheel with the help of the parameters
      speed_publisher( x1, x2, x3, vel_1, vel_2, vel_3);
      }
      else if(time_t >= 3){
        printf("loop2");
      // printf(" some time: %f \n",time_t);
      vel_x = 0;  // v = r*Ω*sin(Θ) 
      vel_y = 0.2 ; // v = r*Ω*cos(Θ)
      vel_z = 0;                                // equation for the circle i.e. no rotation
      // printf(" vel_x : %f vel_y : %f  \n", vel_x, vel_y);

      double coefficients[3][4] =  {{     -1    ,       0.5        ,     0.5         , vel_x  },\
                                    {      0    , 	-sqrt(3)/2     ,   sqrt(3)/2     , vel_y  },\
                                    {     1.0   ,       1.0        ,     1.0         , vel_z  }};  // The allocation matrix along with a column of the desired velocities
      
      velocities = findSolution(coefficients);

      vel_1 = velocities[0];
      vel_2 = velocities[1];
      vel_3 = velocities[2];
      printf("vel_1: %f, vel_2: %f, and vel_3: %f \n", vel_1, vel_2, vel_3);
      free(velocities);

      // Now we need to publish all the velocties for the individual wheel with the help of the parameters
      speed_publisher( x1, x2, x3, vel_1, vel_2, vel_3);

      }
      if(time_t >= 6){
        ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, 0);
        ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, 1);
        ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_1, 2);
        vTaskDelete(NULL);
      }
	}
  
}
void app_main()
{
	// xTaskCreate -> Create a new task and add it to the list of tasks that are ready to run
	xTaskCreatePinnedToCore(&stepper_task, "stepper task", 8192, NULL, 1, NULL, taskCore); // Running the task on CORE0 only of the esp32
}