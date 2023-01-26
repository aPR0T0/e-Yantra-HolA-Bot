#include <cmath.h>
#include "sra_board.h"
//#define debug
#include<time.h>

// C Headers
#include <stdio.h>
#include <math.h>

const int stepsPerRevolution = 200;

const int kp_x = 1;
const int kp_y = 1;
const int kp_z = 1;


const int angular_velocity = 0.5 ;// rad/sec
const int k = 1; // Increase this only when u need a larger radius
#define highest_delay = 80000;
#define lowest_delay = 8500;

#define highest_speed = 1000;
#define lowest_speed = 0.1;

// We will be varying the delay to control the speed of the bot

int x1, x2, x3; //These are the delay variables which will help us tweak delay for the speeds of the motors

///////////////////////////////////////////////////////////////////////
/*
  Function   :    Timer
  Arguments  ;    none
  Returns    :    current time
*/

clock_t before = clock(); // Initialzing time

clock_t timer(void *arg){

  // Time function initialization
  clock_t difference = clock() - before;
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
int findSolution(double coeff[3][4])
{     
    int ans[3];
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
    double D  = determinantOfMatrix(d);
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

void speed_publisher(int x1, int x2, int x3, int vel_1, int vel_2, int vel_3){

  // Now we need to transform the velocity in rpm to the delay in microSec
  // Let's consider 10 rpm as the max speed and 0 rpm as the least speed

  if(vel_1 > 1000){
    vel_1 = 1000;
    gpio_set_level(GPIO_NUM_27 , 1); // anticlockwise direction
  }
  else if( vel_1 < -1000){
    vel_1 = -1000;
    gpio_set_level(GPIO_NUM_27 , 0); // clockwise direction
  }

  if(vel_2 > 1000){
    vel_2 = 1000;
    gpio_set_level(GPIO_NUM_32 , 1);
  }
  else if(vel_2 < -1000){
    vel_2 = -1000;
    gpio_set_level(GPIO_NUM_32 , 0);
  }

  if(vel_3 > 1000){
    vel_3 = 1000;
    gpio_set_level(GPIO_NUM_17 , 1);
  }
  else if(vel_3 < -1000){
    vel_3 = -1000;
    gpio_set_level(GPIO_NUM_17 , 0);
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

  if(x1 < 8500){
    x1 = 8500;
    gpio_set_level(GPIO_NUM_4, 0);
    
  }
  else if(x1 > 85000){
    gpio_set_level(GPIO_NUM_4, 1); // Turning off the motor 1 as the delay is too high
  }
  if(x2 < 8500){
    x2 = 8500;
    gpio_set_level(GPIO_NUM_12, 0);
  }
  else if(x2 > 85000){
    gpio_set_level(GPIO_NUM_12, 1); // Turning off the motor 2 as the delay is too high
  }
  if(x3 < 8500){
    x3 = 8500;
    gpio_set_level(GPIO_NUM_25, 0);
  }
  else if(x3 > 85000){
    gpio_set_level(GPIO_NUM_25, 1); // Turning off the motor 3 as the delay is too high
  }

  for(int i = 0 ; i < stepsPerRevolution ; i++){
			gpio_set_level(GPIO_NUM_33, 1);
      gpio_set_level(GPIO_NUM_14, 1);
			gpio_set_level(GPIO_NUM_16, 1);
			ets_delay_us(8500 / portTICK_PERIOD_MS);
      gpio_set_level(GPIO_NUM_33, 0);
      ets_delay_us(x1 / portTICK_PERIOD_MS);
      gpio_set_level(GPIO_NUM_16, 0);
      ets_delay_us(x2 / portTICK_PERIOD_MS);
      gpio_set_level(GPIO_NUM_14, 0);
      ets_delay_us(x3 / portTICK_PERIOD_MS);
  }

}

// ######################################################################################################################### //
// ######################################################################################################################### //


// ######################################################################################################################### //
// ##########################################      Main Function coming in      ############################################ //
// ######################################################################################################################### //

void stepper_task(){

	gpio_config_t io_conf;
    // bit mask for the pins, each bit maps to a GPIO
    // All pin defs according to each motor
    io_conf.pin_bit_mask = (      (1ULL<<GPIO_NUM_27) | (1ULL<<GPIO_NUM_14) | (1ULL<<GPIO_NUM_32) \     
                                | (1ULL<<GPIO_NUM_33) | (1ULL<<GPIO_NUM_16) | (1ULL<<GPIO_NUM_17) \
                                | (1ULL<<GPIO_NUM_4) | (1ULL<<GPIO_NUM_12) | (1ULL<<GPIO_NUM_25));      
    // set gpio mode to input
    io_conf.mode = GPIO_MODE_OUTPUT;
	// enable pull up resistors
    io_conf.pull_up_en = 0;
    // disable pull down resistors
    io_conf.pull_down_en = 1;
    // disable gpio interrupts
    io_conf.intr_type = GPIO_INTR_DISABLE;
  // put your main code here, to run repeatedly:
  float err_x, err_y, err_theta;
  // Now  to know the movements let me define the velocity vectors in x,y and angular velocity vector about z-axis
  // We will calculate the errors along them and then transform these vectors into individual velocities of the wheels
  float vel_x, vel_y, vel_z, vel_1, vel_2, vel_3;
  double coefficients[3][4];  // The allocation matrix along with a column of the desired velcoties
  double velocities[3];       // The final Velocity matrix after the allocation for each wheels
  double rotation_matrix[3][3];

  // Rotation matrix definition just for the next gen code
  double rot_z;
  double theta;
  clock_t time_z;

  rotation_matrix[0] = {  math::cos(rot_z) , math::sin(rot_z) , 0};
  rotation_matrix[1] = { -math::sin(rot_z) , math::cos(rot_z) , 0};
  rotation_matrix[2] = {         0         ,        0         , 1};

  while(1){
      time_z = timer();
      
      theta = angular_velocity*time_z;

      vel_x = k*math::sin(theta); // equation for the circle
      vel_y = k*math::cos(theta); // equation for the circle
      vel_z = 0;                  // equation for the circle i.e. no rotation

      coefficients[0] = {  1      ,       -0.5        ,     -0.5        , vel_x};
      coefficients[1] = {  0      , -cmath::sqrt(3)/2 , cmath::sqrt(3)/2, vel_y};
      coefficients[2] = { -1      ,       -1          ,      -1         , vel_z};
      
      
      velocities[] = findSolution(coefficients);

      vel_1 = velocities[0];
      vel_2 = velocities[1];
      vel_3 = velocities[2];

      // Now we need to publish all the velocties for the individual wheel with the help of the parameters
      speed_publisher(x1,x2,x3, vel_1, vel_2, vel_3);


      // Just to avoid error
      vTaskDelay(10/ portTICK_period_MS);
  }
  
}
void app_main()
{
	// xTaskCreate -> Create a new task and add it to the list of tasks that are ready to run
	xTaskCreate(&stepper_task, "stepper task", 4096, NULL, 1, NULL);
}