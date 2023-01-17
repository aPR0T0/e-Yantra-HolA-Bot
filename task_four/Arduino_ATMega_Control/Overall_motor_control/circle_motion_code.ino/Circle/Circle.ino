
#include <math.h>
#include <cmath.h>

const int dirPinM1 = 8;  // dir pin for motor 1
const int dirPinM2 = 12;
const int dirPinM3 = 9;
const int stepPinM1 = 6; // Step pin for motor 1
const int stepPinM2 = 10;
const int stepPinM3 = 7;

const int enablePinM1 = 11;
const int enablePinM2 = 13;
const int enablePinM3 = 15;
const int stepsPerRevolution = 200;

const int kp_x = 1;
const int kp_y = 1;
const int kp_z = 1;

const int k = 1; // Increase this only when u need a larger radius
unsigned long time;
time = millis();         // this will give us the time elapsed since the 
time = time / 1000;
const int angular_velocity = 0.5 ;// rad/sec
// We will be varying the delay to control the speed of the bot

int x1, x2, x3; //These are the delay variables which will help us tweak delay for the speeds of the motors


//                                                                                            //
// Note: Min delay in MicroSeconds i.e. the highest speeds = 450 -> Max error with constants  //
//                                      the lowest  speeds = 2000 -> Min error with constants //


// Also: Dir pin if high -> Anti-clockwise rotation
//       Dir pin if low  -> Clockwise rotation


void setup() {
  // put your setup code here, to run once:
  pinMode(stepPinM1, OUTPUT);
  pinMode(dirPinM1 , OUTPUT);
  
  pinMode(stepPinM2, OUTPUT);
  pinMode(dirPinM2 , OUTPUT);
  
  pinMode(stepPinM3, OUTPUT);
  pinMode(dirPinM3 , OUTPUT);

  pinMode(enablePinM1, OUTPUT);
  pinMode(enablePinM2, OUTPUT);
  pinMode(enablePinM3, OUTPUT);

}

double modulus(double z){
  if(z<0){
    z = -z;
  }

  return z; 
}

// ##### This part is only for the inverse kinematics to get the solution of for the desired output at any given time ##### //

double determinantOfMatrix(double mat[3][3])
{
    double ans;
    ans = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2])
          - mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0])
          + mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
    return ans;
}

// This function finds the solution of system of
// linear equations using cramer's rule
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
    double D = determinantOfMatrix(d);
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

void speed_publisher(int x1, int x2, int x3, int vel_1, int vel_2, int vel_3){

  // Now we need to transform the velocity in rpm to the delay in microSec
  // Let's consider 10 rpm as the max speed and 0 rpm as the least speed

  if(vel_1 > 5){
    vel_1 = 5;
  }
  else if( vel_1 < -5){
    vel_1 = -5;
  }

  if(vel_2 > 5){
    vel_2 = 5;
  }
  else if(vel_2 < -5){
    vel_2 = -5;
  }

  if(vel_3 > 5){
    vel_3 = 5;
  }
  else if(vel_3 < -5){
    vel_3 = -5;
  }
  
  x1 = 2250 - 450*modulus(vel_1);
  x2 = 2250 - 450*modulus(vel_2);
  x3 = 2250 - 450*modulus(vel_3);

  // Motor1   

  if(vel_1 < 0){
    digitalWrite(dirPinM1, LOW);
    if((x1)< 450){           
      x1 = 450;         // Max speed possibel is at th delay of 450
    }
  }
  else{
    digitalWrite(stepPinM1, HIGH);
    if(vel_1<450){
      x1 = 450;
    }
  }
  if(x1 < 2250){
    for(int x = 0; x < stepsPerRevolution; x++)
    {
      digitalWrite(stepPinM1, HIGH);
      delayMicroseconds(x1);
      digitalWrite(stepPinM2, LOW);
      delayMicroseconds(x1);
    }
  }
  else{
    digitalWrite(enablePinM1, HIGH)
  }
  
  // Motor2

  if(vel_2 < 0){
    digitalWrite(dirPinM2, LOW);
    if(x2<450){
      x2 = 450;
    }
  }
  else{
    digitalWrite(dirPinM2, HIGH);
    if(x2<450){
      x2 = 450;
    }
  }
  if(x2 < 2250){
    for(int x = 0; x < stepsPerRevolution; x++)
    {
      digitalWrite(stepPinM2, HIGH);
      delayMicroseconds(x2);
      digitalWrite(stepPinM2, LOW);
      delayMicroseconds(x2);
    }
  }
  else{
    digitalWrite(enablePinM2, HIGH); // Velocity is zero
  }
  
  // Motor3
  
  if(vel_3 < 0){
    digitalWrite(dirPinM3, LOW);
    if(x3<450){
      x3 = 450;
    }
  }
  else{
    digitalWrite(dirPinM3, HIGH);

    if(x3<450){
      x3 = 450;
    }
  }
  if(x3 < 2250){
    for(int x = 0; x < stepsPerRevolution; x++)
    {
      digitalWrite(stepPinM3, HIGH);
      delayMicroseconds(x3);
      digitalWrite(stepPinM3, LOW);
      delayMicroseconds(x3);
    }
  }
  else{
    digitalWrite(enablePinM3, HIGH);
  }
}

// ######################################################################################################################### //
// ######################################################################################################################### //


// ######################################################################################################################### //
// ##########################################      Main Function coming in      ############################################ //
// ######################################################################################################################### //

void loop() {
  // put your main code here, to run repeatedly:
  float err_x, err_y, err_theta;
  // Now  to know the movements let me define the velocity vectors in x,y and angular velocity vector about z-axis
  // We will calculate the errors along them and then transform these vectors into individual velocities of the wheels
  float vel_x, vel_y, vel_z, vel_1, vel_2, vel_3;

  vel_x = k*math::sin(angular_velocity*time); // equation for the circle
  vel_y = k*math::cos(angular_velocity*time); // equation for the circle
  vel_z = 0;                                  // equation for the circle i.e. no rotation

  double coefficients[3][4] ;
  coefficients[0] = {1,-0.5,-0.5, vel_x};
  coefficients[1] = {0, -cmath::sqrt(3)/2, cmath::sqrt(3)/2, vel_y};
  coefficients[2] = {-1,-1,-1, vel_z};
  
  double velocities[3];
  velocities[] = findSolution(coefficients);

  vel_1 = velocities[0];
  vel_2 = velocities[1];
  vel_3 = velocities[2];

  // Now we need to publish all the velocties for the individual wheel with the help of the parameters
  speed_publisher(x1,x2,x3, vel_1, vel_2, vel_3);
  
}
