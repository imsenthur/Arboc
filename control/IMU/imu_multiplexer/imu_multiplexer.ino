// ================================================================
// ===               IMU 6050 KALMAN FILTER CODE                ===
// ===               CODE BY MANU AATITYA R P                   ===
// ===               lANGUAGE : ARDUINO                         ===
// ================================================================


// ===============================================================================
// ===               NECESSARY VARIABLES AND LIBRARIES INCLUDED                ===
// ===============================================================================

#include<Wire.h>

# define Number_of_IMU 6

# define TCAADDR 0x70 // Address of Multiplexer

# define MPU_address 0x68 // Address of imu 1
//# define MPU_address2 0x69 // Address of imu 2

int counter = 0;

// Define necessary variables
int16_t ax,ay,az,wx,wy,wz;  // accelerometer and gyroscope variables
// first index is the number of imu's
double g_predicted[Number_of_IMU][2][3] = {0}; // g values for accelerometer and gyroscope 
// first index is the number of imu's
double theta_predicted[Number_of_IMU][2][3] = {0}; // theta values of accelerometer and gyroscope
double time_step[Number_of_IMU],previous_time[Number_of_IMU],time_elapsed[Number_of_IMU]; // time variables for gyroscope
double g[Number_of_IMU][3] = {0};  // Matrix for g values row indicates imu number
double theta[Number_of_IMU][3] = {0} ; // final theta values

// Kalman Filter Coefficients and Matrices  Error Covariance matrices for the filters 
// first dimension indicates number of imu's
double p[Number_of_IMU][3][2][2] = {0};

// Kalman gain coefficients for the filters
// Row indicates number of imus's
double k[Number_of_IMU][3][2] = {0};

// Bias Variables 
double bias[Number_of_IMU][3] = {0};

// Kalman Coefficients for Covariance matrices
double angle_bias = 1.50; //0.003  // 0.05 made the values closer
double measurement_bias = 1.50; //0.003
double covariance_measure = 0.03; // 0.003 

// ================================================================
// ===               IMU SELECTION ROUTINE                      ===
// ================================================================

void select_imu(uint8_t i)
{
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission(); 
}

// ======================================================================
// ===               CONFIGURE I2C MULTIPLEXER ROUTINE                ===
// ======================================================================

// Configuring first MPU
void configure_multiplexer(int i)//,const int MPU_address)
{
    Wire.begin(); // initialize I2C
    select_imu(i);
    Wire.beginTransmission(MPU_address); // start test transmission
    Wire.write(0x6B);  // power management register
    Wire.write(0x00);     // pull down logic and preventing from going to sleep mode
    Wire.endTransmission(true); // end test transmission

    Wire.beginTransmission(MPU_address); // start test transmission
    Wire.write(0x1B);  //gyro specifications register
    Wire.write(0x00);     // pull down logic and preventing from going to sleep mode 
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_address); // start test transmission
    Wire.write(0x1C);  // accelerometer specifications register
    Wire.write(0x00);     // pull down logic and preventing from going to sleep mode   +- 2g configuration
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_address);
    Wire.write(0x19); // Accessing register to change sampling rate 
    Wire.write(19);     // Setting the sampling rate to 50 hz based on the formula : sampling rate = (Output_rate)/(1+Sampling_rate_divider)
                        // this is reasonable as each motion processing requires about 3 ms to process so for a total of 6 imus 18 ms 
                        // but our update rate is 50hz giving us 20ms between two updates.
    Wire.endTransmission(true);
}

// ================================================================
// ===               KALMAN FUNCTION ROUTINE                    ===
// ================================================================

// Kalman Function 
void Kalman(double z[3],double theta_dot[3],double dt,int imu_number)
{
  double angle[3] = {0};
  double theta_unbiased[3] = {0};
  for(int i =0 ;i < 3;i++)
  {
    theta_unbiased[i] = theta_dot[i] - bias[imu_number][i];
    angle[i] = theta_unbiased[i] * dt;
    p[imu_number][i][0][0] += dt * (dt*p[imu_number][i][1][1] - p[imu_number][i][0][1] - p[imu_number][i][1][0] + angle_bias);
    p[imu_number][i][0][1] -= dt * p[imu_number][i][1][1];
    p[imu_number][i][1][0] -= dt * p[imu_number][i][1][1];
    p[imu_number][i][1][1] += measurement_bias * dt;
    double S = p[imu_number][i][0][0] + covariance_measure;
    k[imu_number][i][0] = p[imu_number][i][0][0] / S;
    k[imu_number][i][1] = p[imu_number][i][1][0] / S;
    double y = z[i] - angle[i];
    angle[i] += k[imu_number][i][0] * y;
    bias[imu_number][i] = k[imu_number][i][0] * y;

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
void getDataFromMPU()
{
  for(int i = 0 ;i < Number_of_IMU ; i++)
  {
    
      configure_multiplexer(i);//,MPU_address);
      select_imu(i);
    /*
      configure_multiplexer(6);//,MPU_address);
      select_imu(6);
    */
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

    // Serial.print(ax);Serial.print(" |");Serial.print(ay);Serial.print(" |");Serial.print(az);Serial.println(" |");
    // time elapsed since taking gyroscope readings
    previous_time[i] = time_step[i];
    time_step[i] = millis();
    time_elapsed[i] = (time_step[i] - previous_time[i] ) / 1000.0;

    // Calculating g values from the raw data obtained from MPU

    // Accelerometer
    // sensitivity scale factor of the accelerometer is 16384 counts/g
    g_predicted[i][0][0] = ax/(16384.0);
    g_predicted[i][0][1] = ay/(16384.0);
    g_predicted[i][0][2] = az/(16384.0);

    //Serial.print(g_predicted[i][0][0]);Serial.print(" | ");Serial.print(g_predicted[i][0][1]);Serial.print(" | ");Serial.print(g_predicted[i][0][2]);Serial.println(" | ");
    // theta values from accelerometer
    
    theta_predicted[i][0][0] = (180/3.141592) * atan(g_predicted[i][0][0] / sqrt(square(g_predicted[i][0][1]) + square(g_predicted[i][0][2]))); 
    theta_predicted[i][0][1] = (180/3.141592) * atan(g_predicted[i][0][1] / sqrt(square(g_predicted[i][0][0]) + square(g_predicted[i][0][2])));
    theta_predicted[i][0][2] = (180/3.141592) * atan(sqrt(square(g_predicted[i][0][1]) + square(g_predicted[i][0][0])) / g_predicted[i][0][2]);
    
    // Serial.print(theta_predicted[i][0][0]);Serial.print(" | ");Serial.print(theta_predicted[i][0][1]);Serial.print(" | ");Serial.print(theta_predicted[i][0][2]);Serial.println(" | ");
    
    // Gyroscope
    // Sensitivity scale factor of gyroscope is 131 radians/s
    g_predicted[i][1][0] = wx / (131.0);
    g_predicted[i][1][1] = wy / (131.0);
    g_predicted[i][1][2] = wz / (131.0);
  
    // Theta values for gyroscope
      theta_predicted[i][1][0] = g_predicted[i][1][0];
      theta_predicted[i][1][1] = g_predicted[i][1][1];
      theta_predicted[i][1][2] = g_predicted[i][1][2];

    // Serial.print(theta_predicted[i][0][0]);Serial.print(" ");
    // Serial.print(theta_predicted[i][1][0]);Serial.print(" | ");Serial.print(theta_predicted[i][1][1]);Serial.print(" | ");Serial.print(theta_predicted[i][0][2]);Serial.println(" | ");
    Kalman(theta_predicted[i][0],theta_predicted[i][1],time_elapsed[i],i);
    //delay(333); // To avoid too much results in the serial monitor;
    //Serial.println();
    // Serial.print(theta[0][0]);Serial.print(" | ");Serial.print(theta[0][1]);Serial.print(" | ");Serial.println(theta[0][2]);
    // Serial.println('\n');
    //delay(333);
  }
}

// ================================================================
// ===                      SETUP  ROUTINE                      ===
// ================================================================

// Setup loop
void setup()
{
  for(int i=0;i < Number_of_IMU;i++)
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
  getDataFromMPU();
  for(int i = 1 ;i < Number_of_IMU ; i++)
  {
      /*          
          Debugging Code 
         // Serial.print("MPU ");Serial.print(i);Serial.print(" with respect to MPU ");Serial.println(0);
        // Serial.println(theta[i][0]);
      */
    /*
     *  Code to compute relative angle between subsequent links updated
     */
    
    Serial.print("MPU ");Serial.print(i);Serial.print(" with respect to MPU ");Serial.println(i-1);
    Serial.print("RelativeTheta X = "); Serial.print(theta[i][0] - theta[i-1][0]);
    Serial.print(" | RelativeTheta_Y = "); Serial.print(theta[i][0] - theta[i-1][1]);
    Serial.print(" | RelativeTheta_Z = "); Serial.println(theta[i][0] - theta[i-1][2]);
   
  }
  Serial.println('\n');
}

