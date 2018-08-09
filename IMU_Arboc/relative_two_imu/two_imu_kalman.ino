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
double theta_x[2],theta_y[2],theta_z[2]; // final theta values

// Kalman Filter Coefficients and Matrices
// Error Covariance matrices for the first filters
double p1x[2][2] = {0};
double p1y[2][2] = {0};
double p1z[2][2] = {0};

// Kalman gain coefficients for the first filters
double k1x[2] ={0};
double k1y[2] ={0};
double k1z[2] ={0};

// Bias Variables 
double bias_1_x = 0,bias_1_y = 0,bias_1_z = 0;

// Error Covariance matrices for the second filters
double p2x[2][2] = {0};
double p2y[2][2] = {0};
double p2z[2][2] = {0};

// Kalman gain coefficients for the second filters
double k2x[2] ={0};
double k2y[2] ={0};
double k2z[2] ={0};

// Bias Variables 
double bias_2_x = 0,bias_2_y = 0,bias_2_z = 0;

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
void getDataFromMPU1(const int MPU_address)
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

  kalman_1_x(theta_a_x,theta_g_x,time_elapsed1);
  kalman_1_y(theta_a_y,theta_g_y,time_elapsed1);
  kalman_1_z(theta_a_z,theta_g_z,time_elapsed1);

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

void kalman_1_x(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_1_x;
  angle += theta_unbiased * dt;
  p1x[0][0] += dt * (dt*p1x[1][1] - p1x[0][1] - p1x[1][0] + angle_bias);
  p1x[0][1] -= dt * p1x[1][1];
  p1x[1][0] -= dt * p1x[1][1];
  p1x[1][1] += measurement_bias * dt;

  double S = p1x[0][0] + covariance_measure;
  k1x[0] = p1x[0][0] / S;
  k1x[1] = p1x[1][0] / S;
  double y = z - angle;
  angle += k1x[0] * y;
  bias_1_x = k1y[0] * y;

  p1x[0][0] -= k1x[0] * p1x[0][0];
  p1x[0][1] -= k1x[0] * p1x[0][1];
  p1x[1][0] -= k1x[1] * p1x[0][0];
  p1x[1][1] -= k1x[1] * p1x[0][1];
  theta_x[0] = angle;
  //Serial.print("Theta X = "); Serial.print(angle);
}

void kalman_1_y(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_1_y;
  angle += theta_unbiased * dt;
  p1y[0][0] += dt * (dt*p1y[1][1] - p1y[0][1] - p1y[1][0] + angle_bias);
  p1y[0][1] -= dt * p1y[1][1];
  p1y[1][0] -= dt * p1y[1][1];
  p1y[1][1] += measurement_bias * dt;

  double S = p1y[0][0] + covariance_measure;
  k1y[0] = p1y[0][0] / S;
  k1y[1] = p1y[1][0] / S;
  double y = z - angle;
  angle += k1x[0] * y;
  bias_1_y = k1y[0] * y;

  p1y[0][0] -= k1y[0] * p1y[0][0];
  p1y[0][1] -= k1y[0] * p1y[0][1];
  p1y[1][0] -= k1y[1] * p1y[0][0];
  p1y[1][1] -= k1y[1] * p1y[0][1];
  theta_y[0] = angle;
  //Serial.print(" | Theta_Y = "); Serial.print(angle);
}

void kalman_1_z(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_1_z;
  angle += theta_unbiased * dt;
  p1z[0][0] += dt * (dt*p1z[1][1] - p1z[0][1] - p1z[1][0] + angle_bias);
  p1z[0][1] -= dt * p1z[1][1];
  p1z[1][0] -= dt * p1z[1][1];
  p1z[1][1] += measurement_bias * dt;

  float S = p1z[0][0] + covariance_measure;
  k1z[0] = p1z[0][0]/ S;
  k1z[1] = p1z[1][0]/ S;
  float y = z - angle;
  angle += k1z[0] * y;
  bias_1_z = k1z[0] * y;

  p1z[0][0] -= k1z[0] * p1z[0][0];
  p1z[0][1] -= k1z[0] * p1z[0][1];
  p1z[1][0] -= k1z[1] * p1z[0][0];
  p1z[1][1] -= k1z[1] * p1z[0][1];
  theta_z[0] = angle;
  //Serial.print(" | Theta_Y = "); Serial.println(angle);
}

// Function to retrieve data from MPU
void getDataFromMPU2(const int MPU_address)
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
  previous_time2 = time2;
  time2 = millis();
  time_elapsed2 = (time2 - previous_time2 ) / 1000.0;
  
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

  kalman_2_x(theta_a_x,theta_g_x,time_elapsed1);
  kalman_2_y(theta_a_y,theta_g_y,time_elapsed1);
  kalman_2_z(theta_a_z,theta_g_z,time_elapsed1);
}

void kalman_2_x(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_2_x;
  angle += theta_unbiased * dt;
  p2x[0][0] += dt * (dt*p2x[1][1] - p2x[0][1] - p2x[1][0] + angle_bias);
  p2x[0][1] -= dt * p2x[1][1];
  p2x[1][0] -= dt * p2x[1][1];
  p2x[1][1] += measurement_bias * dt;

  double S = p2x[0][0] + covariance_measure;
  k2x[0] = p2x[0][0] / S;
  k2x[1] = p2x[1][0] / S;
  double y = z - angle;
  angle += k2x[0] * y;
  bias_2_x = k2y[0] * y;

  p2x[0][0] -= k2x[0] * p2x[0][0];
  p2x[0][1] -= k2x[0] * p2x[0][1];
  p2x[1][0] -= k2x[1] * p2x[0][0];
  p2x[1][1] -= k2x[1] * p2x[0][1];
  theta_x[1] = angle;
  //Serial.print("Theta X = "); Serial.print(angle);
}

void kalman_2_y(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_2_y;
  angle += theta_unbiased * dt;
  p2y[0][0] += dt * (dt*p2y[1][1] - p2y[0][1] - p2y[1][0] + angle_bias);
  p2y[0][1] -= dt * p2y[1][1];
  p2y[1][0] -= dt * p2y[1][1];
  p2y[1][1] += measurement_bias * dt;

  double S = p2y[0][0] + covariance_measure;
  k2y[0] = p2y[0][0] / S;
  k2y[1] = p2y[1][0] / S;
  double y = z - angle;
  angle += k2x[0] * y;
  bias_2_y = k2y[0] * y;

  p2y[0][0] -= k2y[0] * p2y[0][0];
  p2y[0][1] -= k2y[0] * p2y[0][1];
  p2y[1][0] -= k2y[1] * p2y[0][0];
  p2y[1][1] -= k2y[1] * p2y[0][1];
  theta_y[1] = angle;
  //Serial.print(" | Theta_Y = "); Serial.print(angle);
}

void kalman_2_z(double z,double theta_dot,double dt)
{
  double angle = 0;
  double theta_unbiased = theta_dot - bias_2_z;
  angle += theta_unbiased * dt;
  p2z[0][0] += dt * (dt*p2z[1][1] - p2z[0][1] - p2z[1][0] + angle_bias);
  p2z[0][1] -= dt * p2z[1][1];
  p2z[1][0] -= dt * p2z[1][1];
  p2z[1][1] += measurement_bias * dt;

  float S = p2z[0][0] + covariance_measure;
  k2z[0] = p2z[0][0]/ S;
  k2z[1] = p2z[1][0]/ S;
  float y = z - angle;
  angle += k2z[0] * y;
  bias_2_z = k2z[0] * y;

  p2z[0][0] -= k2z[0] * p2z[0][0];
  p2z[0][1] -= k2z[0] * p2z[0][1];
  p2z[1][0] -= k2z[1] * p2z[0][0];
  p2z[1][1] -= k2z[1] * p2z[0][1];
  theta_z[1] = angle;
  //Serial.print(" | Theta_Y = "); Serial.println(angle);
}


// Main loop
void loop()
{ 
  delay(333); // To prevent too much display on Serial Monitor
  Serial.println("Relative Angles are :");
  getDataFromMPU1(MPU_address1);
  getDataFromMPU2(MPU_address2);
  Serial.print("Theta X = "); Serial.print(theta_x[1] - theta_x[0]);
  Serial.print(" | Theta_Y = "); Serial.print(theta_y[1] - theta_y[0]);
  Serial.print(" | Theta_z = "); Serial.println(theta_z[1] - theta_z[0]);
}

