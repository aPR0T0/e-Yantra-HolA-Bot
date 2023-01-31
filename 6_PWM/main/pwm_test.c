#include "sra_board.h"
//#define debug
#include <time.h>
// C Headers
#include <stdio.h>
#include <math.h>

// Timer functions in FreeRTOS
#include <freertos/timers.h>

const int stepsPerRevolution = 200;

const int kp_x = 8000;
const int kp_y = 1;
const int kp_z = 1;


const int angular_velocity = 16;// rad/sec -> theta = s/R
const int k = 1; // Increase this only when u need a larger radius

#define highest_delay  3000
#define lowest_delay  100

#define highest_speed  13
#define lowest_speed  0

// We will be varying the delay to control the speed of the bot

int x1, x2, x3; //These are the delay variables which will help us tweak delay for the speeds of the motors
int64_t before;
int64_t motor_one_prev_time;
int64_t motor_two_prev_time;
int64_t motor_three_prev_time;
int count = 0;

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
    ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
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
void speed_publisher(int x1, int x2, int x3, int vel_1, int vel_2, int vel_3){	// Publishes Speed according to the given body frame velocities

  // Now we need to transform the velocity in rpm to the delay in microSec
  // Let's consider 7 rpm as the max speed and 0 rpm as the least speed
  int64_t motor_one_curr_time   = esp_timer_get_time();
  int64_t motor_two_curr_time   = esp_timer_get_time();
  int64_t motor_three_curr_time = esp_timer_get_time();

  if(vel_1 > 0){
    gpio_set_level(GPIO_NUM_27 , 1); // anticlockwise direction
    if(vel_1>7){
      vel_1 = 7;
    }
  }
  else if( vel_1 <= 0){
    gpio_set_level(GPIO_NUM_27 , 0); // clockwise direction
    if(vel_1 < -7){
      vel_1 = -7;
    }
  }

  if(vel_2 > 0){
    gpio_set_level(GPIO_NUM_32 , 1);
    if(vel_2 > 7){
      vel_2 = 7;
    }
  }
  else if(vel_2 <= 0){
    gpio_set_level(GPIO_NUM_32 , 0);
    if(vel_2 < -7){
      vel_2 = -7;
    }
  }

  if(vel_3 > 0){
    gpio_set_level(GPIO_NUM_17 , 1);
    if(vel_3 > 7){
      vel_3 = 7;
    }
  }
  else if(vel_3 <= 0){
    gpio_set_level(GPIO_NUM_17 , 0);
    if(vel_3 < -7){
      vel_3 = -7;
    }
  }

  // Now, just mapping the velocities to the delays

  vel_1 = modulus(vel_1);   // Direction pins are already set 
  vel_2 = modulus(vel_2);   // Direction pins are already set 
  vel_3 = modulus(vel_3);   // Direction pins are already set 
  
  x1 = map(vel_1,lowest_speed, highest_speed, lowest_delay, highest_delay);
  x2 = map(vel_2,lowest_speed, highest_speed, lowest_delay, highest_delay);
  x3 = map(vel_3,lowest_speed, highest_speed, lowest_delay, highest_delay);

  x1 = highest_delay - x1;  // Inverting the delay because of the inverse relation of the speeds and the delays
  x2 = highest_delay - x2;  // Inverting the delay because of the inverse relation of the speeds and the delays
  x3 = highest_delay - x3;  // Inverting the delay because of the inverse relation of the speeds and the delays  

  if(x1 < 100){
    x1 = 100;
    
  }
  if(x2 < 100){
    x2 = 100;
  }
  if(x3 < 100){
    x3 = 100;
  }
  printf("x1 : %d x2 : %d and x3 : %d\n",x1, x2, x3);
  // printf("x1 : %lld x2 : %lld and x3 : %lld\n", (motor_one_curr_time- motor_one_prev_time), (motor_two_curr_time - motor_three_prev_time), (motor_three_curr_time - motor_three_prev_time));

  gpio_set_level(GPIO_NUM_33, 0);
  gpio_set_level(GPIO_NUM_14, 0);
  gpio_set_level(GPIO_NUM_16, 0);
  if((motor_one_curr_time - motor_one_prev_time) >= x1 && x1 < 2800){  // Instead of using for loop we are just checking if the time of the signal high is just enough so that steps are generated only in a given interval of time or delay
    gpio_set_level(GPIO_NUM_33, 1);
    // printf("motor_one_speed_published ");
    motor_one_prev_time = motor_one_curr_time;
  }
  if((motor_two_curr_time - motor_two_prev_time) >= x2 && x2 < 2800){
    gpio_set_level(GPIO_NUM_14, 1);
    // printf("motor_two_speed_published ");
    motor_two_prev_time = motor_two_curr_time;
  }
  if((motor_three_curr_time - motor_three_prev_time) >= x3 && x3 < 2800){
    gpio_set_level(GPIO_NUM_16, 1);
    // printf("motor_three_speed_published\n");
    motor_three_prev_time = motor_three_curr_time;
  }
}

// ######################################################################################################################### //
// ######################################################################################################################### //


// ######################################################################################################################### //
// ##########################################      Main Function coming in      ############################################ //
// ######################################################################################################################### //

void stepper_task(void *arg){

		gpio_config_t io_conf;
    // bit mask for the pins, each bit maps to a GPIO
    // All pin defs according to each motor
    io_conf.pin_bit_mask = ((1ULL<<GPIO_NUM_27) | (1ULL<<GPIO_NUM_14) | (1ULL<<GPIO_NUM_32) | (1ULL<<GPIO_NUM_33) | (1ULL<<GPIO_NUM_16) | (1ULL<<GPIO_NUM_17));      
    // set gpio mode to input
    io_conf.mode  = GPIO_MODE_OUTPUT;
	  // enable pull up resistors
    io_conf.pull_up_en   = 0;
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
  // put your main code here, to run repeatedly:

	// float err_x, err_y, err_theta;

	// Now  to know the movements let me define the velocity vectors in x,y and angular velocity vector about z-axis
	// We will calculate the errors along them and then transform these vectors into individual velocities of the wheels
	double vel_x, vel_y, vel_z, vel_1, vel_2, vel_3;
	double *velocities;       // The final Velocity matrix after the allocation for each wheels

	// Rotation matrix definition just for the next gen code
	double rot_z = 0;
	double theta;
	int64_t time_z, time_t;

	double rotation_matrix[3][3] = {{     cos(rot_z)    ,    sin(rot_z)    , 0},\
                                  {    -sin(rot_z)    ,    cos(rot_z)    , 0},\
                                  {         0         ,        0         , 1}};
  
  motor_one_prev_time   = esp_timer_get_time();
  motor_two_prev_time   = esp_timer_get_time();
  motor_three_prev_time = esp_timer_get_time();
  
	while(1){
		time_z = timer();
    
    time_t = time_z / 1000000; // As time was in microseconds
		// printf(" some time: %ld\n",time_z);

		theta = angular_velocity*time_t;
    theta = theta*(3.14159265358979323846/180);
    // printf(" angle : %f \n", theta);

    if(theta >= 6.28){
      vTaskDelete(NULL);
    }
    vel_x = k*(angular_velocity)*sin(theta);  // v = r*Ω*sin(Θ) 
    vel_y = k*(angular_velocity)*cos(theta);  // v = r*Ω*cos(Θ)
		vel_z = 0;            // equation for the circle i.e. no rotation

		double coefficients[3][4] =  {{      1    ,       -0.5        ,     -0.5         , vel_x  },\
                                  {      0    , 	-sqrt(3)/2      ,    sqrt(3)/2     , vel_y  },\
                                  {    -1.0   ,       -1.0        ,     -1.0         , vel_z  }};  // The allocation matrix along with a column of the desired velocities
		
		velocities = findSolution(coefficients);

		vel_1 = velocities[0];
		vel_2 = velocities[1];
		vel_3 = velocities[2];
    printf("vel_1: %f, vel_2: %f, and vel_3: %f ", vel_1, vel_2, vel_3);
    free(velocities);

		// Now we need to publish all the velocties for the individual wheel with the help of the parameters
		speed_publisher( x1, x2, x3, vel_1, vel_2, vel_3);


		// Just to avoid error
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
  
}
void app_main()
{
	// xTaskCreate -> Create a new task and add it to the list of tasks that are ready to run
	xTaskCreate(&stepper_task, "stepper task", 4096, NULL, 1, NULL);
}