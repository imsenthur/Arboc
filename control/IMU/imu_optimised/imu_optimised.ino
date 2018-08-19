// ================================================================
// ===               IMU 6050 KALMAN FILTER CODE                ===
// ===               CODE BY MANU AATITYA R P                   ===
// ===               lANGUAGE : ARDUINO                         ===
// ================================================================


// This contains the basic code to get values from 2 MPU 6050 simultaneously

// ===============================================================================
// ===               NECESSARY VARIABLES AND LIBRARIES INCLUDED                ===
// ===============================================================================

#include<Wire.h>

# define Number_of_IMU 2

# define TCAADDR 0x70 //Address of Multiplexer

extern "C" {
include "utility/twi.h" // from Wire library, so we can do bus scanning

}

// Define necessary variables
const int MPU_address[Number_of_IMU] = {0x68 , 0x69};  // I2C address of the MPUs
int16_t ax,ay,az,wx,wy,wz;  // accelerometer and gyroscope variables
// first index is the number of imu's
double g_predicted[Number_of_IMU][2][3] = {0}; // g values for accelerometer and gyroscope 
// first index is the number of imu's
double theta_predicted[Number_of_IMU][2][3] = {0}; // theta values of accelerometer and gyroscope
double time_step[Number_of_IMU],previous_time[Number_of_IMU],time_elapsed[Number_of_IMU]; // time variables for gyroscope
int gyro_count[Number_of_IMU] = {1,1}; // count of gyro values 
double g[Number_of_IMU][3] = {0};  // Matrix for g values row indicates imu number
double theta[Number_of_IMU][3] = {0} ; // final theta values

// Kalman Filter Coefficients and Matrices  Error Covariance matrices for the filters 
// first dimension indicates number of imu's
double p[Number_of_IMU][3][2][2] = {0};

// Kalman gain coefficients for the filters
// Row indicates number of imus's
double k[Number_of_IMU][3][2] = {0};

// Bias Variables 
double bias_x[Number_of_IMU] = {0},bias_y[Number_of_IMU] = {0},bias_z[Number_of_IMU] = {0};

// Kalman Coefficients for Covariance matrices
double angle_bias[Number_of_IMU] = {0.001,0.001}; 
double measurement_bias[Number_of_IMU] = {0.003,0.003}; 
double covariance_measure[Number_of_IMU] = {0.003,0.003}; 


// ======================================================================
// ===               CONFIGURE I2C MULTIPLEXER ROUTINE                ===
// ======================================================================

// Configuring first MPU
void configure_multiplexer()
{
  for(int i=0;i < Number_of_IMU;i++)
  {
    Wire.begin(); // initialize I2C
    Wire.beginTransmission(MPU_address[i]); // start test transmission
    Wire.write(0x6B);  // write into register
    Wire.write(0);     // pull down logic 
    Wire.endTransmission(true); // end test transmission
  }
  
}

// ================================================================
// ===               KALMAN FUNCTION ROUTINE                    ===
// ================================================================

// Kalman Function 
void Kalman(double z[3],double theta_dot[3],double dt,int imu_number)
{
  double angle[3] = {0};
  double theta_unbiased[3] = {0};
  for(int i =0 ;i<3;i++)
  {
    theta_unbiased[i] = theta_dot[i] - bias_x[i];
    angle[i] = theta_unbiased[i] * dt;
    p[imu_number][i][0][0] += dt * (dt*p[imu_number][i][1][1] - p[imu_number][i][0][1] - p[imu_number][i][1][0] + angle_bias[i]);
    p[imu_number][i][0][1] -= dt * p[imu_number][i][1][1];
    p[imu_number][i][1][0] -= dt * p[imu_number][i][1][1];
    p[imu_number][i][1][1] += measurement_bias[i] * dt;
    double S = p[imu_number][i][0][0] + covariance_measure[i];
    k[imu_number][i][0] = p[imu_number][i][0][0] / S;
    k[imu_number][i][1] = p[imu_number][i][1][0] / S;
    double y = z[i] - angle[i];
    angle[i] += k[imu_number][i][0] * y;
    bias_x[i] = k[imu_number][i][0] * y;

    p[imu_number][i][0][0] -= k[imu_number][i][0] * p[imu_number][i][0][0];
    p[imu_number][i][0][1] -= k[imu_number][i][0] * p[imu_number][i][0][1];
    p[imu_number][i][1][0] -= k[imu_number][i][1] * p[imu_number][i][0][0];
    p[imu_number][i][1][1] -= k[imu_number][i][1] * p[imu_number][i][0][1];
    theta[imu_number][i] = angle[i];
  }
} 

// ================================================================
// ===               DATA RETRIEVAL ROUTINE                     ===
// ================================================================

// Function to retrieve data from MPU
void getDataFromMPU(const int MPU_address)
{
  int i;
  Wire.beginTransmission(MPU_address);
  Wire.write(0x3B);   // register to access acceleration x high 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address,6);
  while(Wire.available() < 6);
  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  // Leaving Register which gets Temperature values
  
  Wire.beginTransmission(MPU_address);
  Wire.write(0x43);   // Register to access gyroscope x high
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_address,6);
  while(Wire.available() < 6);
  wx = Wire.read() << 8 | Wire.read();
  wy = Wire.read() << 8 | Wire.read();
  wz = Wire.read() << 8 | Wire.read();

  if (MPU_address == 0x68)
  {
    // time elapsed since taking gyroscope readings
    previous_time[0] = time_step[0];
    time_step[0] = millis();
    time_elapsed[0] = (time_step[0] - previous_time[0] ) / 1000.0;
    i = 0;
  }
  else
  {
    // time elapsed since taking gyroscope readings
    previous_time[1] = time_step[1];
    time_step[1] = millis();
    time_elapsed[1] = (time_step[1] - previous_time[1] ) / 1000.0;
    i =1;
  }  
  // Calculating g values from the raw data obtained from MPU

  // Accelerometer
  // sensitivity scale factor of the accelerometer is 16384 counts/g
  g_predicted[i][0][0] = ax/(16384.0);
  g_predicted[i][0][1] = ay/(16384.0);
  g_predicted[i][0][2] = az/(16384.0);

  // theta values from accelerometer
  theta_predicted[i][0][0] = (180/3.141592) * atan(g_predicted[i][0][0] / sqrt(square(g_predicted[i][0][1]) + square(g_predicted[i][0][2]))); 
  theta_predicted[i][0][1] = (180/3.141592) * atan(g_predicted[i][0][1] / sqrt(square(g_predicted[i][0][0]) + square(g_predicted[i][0][2])));
  theta_predicted[i][0][2] = (180/3.141592) * atan(sqrt(square(g_predicted[i][0][1]) + square(g_predicted[i][0][0])) / g_predicted[i][0][2]);

  // Gyroscope
  // Sensitivity scale factor of gyroscope is 131 radians/s
  g_predicted[i][1][0] = wx / (131.0);
  g_predicted[i][1][1] = wy / (131.0);
  g_predicted[i][1][2] = wz / (131.0);

  // Theta values for gyroscope
  theta_predicted[i][1][0] = g_predicted[i][1][0];
  theta_predicted[i][1][1] = g_predicted[i][1][1];
  theta_predicted[i][1][2] = g_predicted[i][1][2];

  Kalman(theta_predicted[i][0],theta_predicted[i][1],time_elapsed[i],i);
  
}

// ================================================================
// ===                      SETUP  ROUTINE                      ===
// ================================================================

// Setup loop
void setup()
{
  configure_multiplexer();
  for(int i=0;i<2;i++)
  {
    time_step[i] = millis();
  }
  Serial.begin(9600); // set baud rate to 9600 for serial communication
}

// ================================================================
// ===                     MAIN ROUTINE                         ===
// ================================================================

// Main loop
void loop()
{ 
  Serial.println("MPU 1");
  for(int i = 0 ;i < Number_of_IMU ; i++)
  {
    Serial.print("MPU");
    Serial.println(i+1);
    getDataFromMPU(MPU_address[i]);
  }
  delay(333); // To avoid too much results in the serial monitor
  Serial.print("RelativeTheta X = "); Serial.print(theta[1][0] - theta[0][0]);
  Serial.print(" | RelativeTheta_Y = "); Serial.print(theta[1][1] - theta[0][1]);
  Serial.print(" | RelativeTheta_Z = "); Serial.println(theta[1][2] - theta[0][2]);

}

