//<-------------------------------------------->
//<-------IMU 6050 kalman Filter code---------->
//<-------Code By : Manu Aatitya R P----------->
//<-------Language : Arduino------------------->
//<-------------------------------------------->

// This contains the basic code to get values from 2 MPU 6050 simultaneously

// Include the necessary libraries
#include<Wire.h>

// Define necessary variables
const int MPU_address1 = 0x68;  // I2C address of the MPU-6050 1
const int MPU_address2 = 0x69; // I2C address for MPU-6050 2
int16_t ax,ay,az,wx,wy,wz;  // accelerometer and gyroscope variables
double gax,gay,gaz,ggx,ggy,ggz; //g value of accelerometer and gyroscope
double theta_a_x,theta_a_y,theta_a_z,theta_g_x,theta_g_y,theta_g_z; // theta values of accelerometer and gyroscope
double time1,time2,previous_time1,previous_time2,time_elapsed1,time_elapsed2; // time variables for gyroscope
int gyro_count1 = 1;
int gyro_count2 = 1;
double ggx1,ggy1,ggz1; // for the values from gyroscope
double theta_x,theta_y,theta_z; // final theta values

// Kalman Filter Coefficients and Matrices
// Error Covariance matrices for the filters
double px[2][2] = {0};
double py[2][2] = {0};
double pz[2][2] = {0};

// Kalman gain coefficients for the filters
double kx[2] ={0};
double ky[2] ={0};
double kz[2] ={0};

// Bias Variables 
double bias_x = 0,bias_y = 0,bias_z = 0;

// Kalman Coefficients for Covariance matrices
double angle_bias = 0.001;   // 0.001
double measurement_bias = 0.003; // 0.003
double covariance_measure = 0.003; // 0.003

// Configuring first MPU
void configure_MPU1()
{
  Wire.begin(); // initialize I2C
  Wire.beginTransmission(MPU_address1); // start test transmission
  Wire.write(0x6B);  // write into register
  Wire.write(0);     // pull down logic 
  Wire.endTransmission(true); // end test transmission
}

// Configuring second MPU
void configure_MPU2()
{
  Wire.begin(); // initialize I2C
  Wire.beginTransmission(MPU_address2); // start test transmission
  Wire.write(0x6B);  // write into register
  Wire.write(0);     // pull down logic 
  Wire.endTransmission(true); // end test transmission
}

// Setup loop
void setup()
{
  configure_MPU1();
  configure_MPU2();
  time1 = millis();
  time2 = millis();
  Serial.begin(9600); // set baud rate to 9600 for serial communication
}

// Function to retrieve data from MPU
void getDataFromMPU(const int MPU_address)
{
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

  // time elapsed since taking gyroscope readings
  previous_time1 = time1;
  time1 = millis();
  time_elapsed1 = (time1 - previous_time1 ) / 1000.0;
  
  // Calculating g values from the raw data obtained from MPU

  // Accelerometer
  // sensitivity scale factor of the accelerometer is 16384 counts/g
  gax = ax/(16384.0);
  gay = ay/(16384.0);
  gaz = az/(16384.0);

  // theta values from accelerometer
  theta_a_x = (180/3.141592) * atan(gax / sqrt(square(gay) + square(gaz))); 
  theta_a_y = (180/3.141592) * atan(gay / sqrt(square(gax) + square(gaz)));
  theta_a_z = (180/3.141592) * atan(sqrt(square(gay) + square(gax)) / gaz);

  // Gyroscope
  // Sensitivity scale factor of gyroscope is 131 radians/s
  ggx = wx / (131.0);
  ggy = wy / (131.0);
  ggz = wz / (131.0);

  // Theta values for gyroscope
  theta_g_x = ggx;
  theta_g_y = ggy;
  theta_g_z = ggz;

  kalman_x(theta_a_x,theta_g_x,time_elapsed1);
  kalman_y(theta_a_y,theta_g_y,time_elapsed1);
  kalman_z(theta_a_z,theta_g_z,time_elapsed1);

  /*
  // Printing the data in the serial monitor
  Serial.print("Theta X = "); Serial.print(theta_x); 
  Serial.print(" | Theta_Y = "); Serial.print(theta_y);
  Serial.print(" | Theta_Z = "); Serial.println(theta_z);
  */
  /*
  Uncomment these to find the comparison between different filters in action in Serial Plotter
  Serial.print(" | Theta_A_X = "); Serial.print(theta_a_x);
  Serial.print(" | Theta_G_X = "); Serial.println(theta_g_x);
  */
}

void kalman_x(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_x;
  angle += theta_unbiased * dt;
  px[0][0] += dt * (dt*px[1][1] - px[0][1] - px[1][0] + angle_bias);
  px[0][1] -= dt * px[1][1];
  px[1][0] -= dt * px[1][1];
  px[1][1] += measurement_bias * dt;

  double S = px[0][0] + covariance_measure;
  kx[0] = px[0][0] / S;
  kx[1] = px[1][0] / S;
  double y = z - angle;
  angle += kx[0] * y;
  bias_x = ky[0] * y;

  px[0][0] -= kx[0] * px[0][0];
  px[0][1] -= kx[0] * px[0][1];
  px[1][0] -= kx[1] * px[0][0];
  px[1][1] -= kx[1] * px[0][1];
  Serial.print("Theta X = "); Serial.print(angle);
}

void kalman_y(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_y;
  angle += theta_unbiased * dt;
  py[0][0] += dt * (dt*py[1][1] - py[0][1] - py[1][0] + angle_bias);
  py[0][1] -= dt * py[1][1];
  py[1][0] -= dt * py[1][1];
  py[1][1] += measurement_bias * dt;

  double S = py[0][0] + covariance_measure;
  ky[0] = py[0][0] / S;
  ky[1] = py[1][0] / S;
  double y = z - angle;
  angle += kx[0] * y;
  bias_y = ky[0] * y;

  py[0][0] -= ky[0] * py[0][0];
  py[0][1] -= ky[0] * py[0][1];
  py[1][0] -= ky[1] * py[0][0];
  py[1][1] -= ky[1] * py[0][1];
  Serial.print(" | Theta_Y = "); Serial.print(angle);
}

void kalman_z(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_z;
  angle += theta_unbiased * dt;
  pz[0][0] += dt * (dt*pz[1][1] - pz[0][1] - pz[1][0] + angle_bias);
  pz[0][1] -= dt * pz[1][1];
  pz[1][0] -= dt * pz[1][1];
  pz[1][1] += measurement_bias * dt;

  float S = pz[0][0] + covariance_measure;
  kz[0] = pz[0][0]/ S;
  kz[1] = pz[1][0]/ S;
  float y = z - angle;
  angle += kz[0] * y;
  bias_z = kz[0] * y;

  pz[0][0] -= kz[0] * pz[0][0];
  pz[0][1] -= kz[0] * pz[0][1];
  pz[1][0] -= kz[1] * pz[0][0];
  pz[1][1] -= kz[1] * pz[0][1];
  Serial.print(" | Theta_Y = "); Serial.println(angle);
}

// Main loop
void loop()
{ 
  delay(333);
  Serial.println("MPU 1");
  getDataFromMPU(MPU_address1);
  delay(333); // To avoid too much results in the serial monitor
  /*
  Serial.println("MPU 2");
  getDataFromMPU(MPU_address2);
  delay(333); // To avoid too much results in the serial monitor
  */
}

