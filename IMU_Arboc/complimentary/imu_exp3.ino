//<------------------------------------>
//<-------IMU 6050 basic code---------->
//<-------Code By : Manu Aatitya R P--->
//<-------Language : Arduino----------->
//<------------------------------------>

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

// Complimentary filter coefficients
double gyro_coefficient = 0.93,acc_coefficient = 0.07;       // adjust these to vary complimentary filter results

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
  
  if(gyro_count1 == 1)
  {
    theta_g_x = gax;
    theta_g_y = gay;
    theta_g_z = gaz;
    ggx1 = gax;  // theta0 values
    ggy1 = gay;  // theta0 values
    ggz1 = gaz;  // theta0 values
  }
  else
  {
    theta_g_x = ggx1 + (ggx * (time_elapsed1));
    theta_g_y = ggy1 + (ggy * (time_elapsed1));
    theta_g_z = ggz1 + (ggz * (time_elapsed1));
  }
  gyro_count1 +=1;

  // Complimentary filter
  
  theta_x = (gyro_coefficient * theta_g_x) + (acc_coefficient * theta_a_x);
  theta_y = (gyro_coefficient * theta_g_y) + (acc_coefficient * theta_a_y);
  theta_z = (gyro_coefficient * theta_g_z) + (acc_coefficient * theta_a_z);
  
  
  // Printing the data in the serial monitor
  Serial.print("Theta X = "); Serial.print(theta_x); 
  Serial.print(" | Theta_Y = "); Serial.print(theta_y);
  Serial.print(" | Theta_Z = "); Serial.println(theta_z);
  
}

// Main loop
void loop()
{ 
  delay(333);
  Serial.println("MPU 1");
  getDataFromMPU(MPU_address1);
  delay(333); // To avoid too much results in the serial monitor
  
  Serial.println("MPU 2");
  getDataFromMPU(MPU_address2);
  delay(333); // To avoid too much results in the serial monitor
  
}

