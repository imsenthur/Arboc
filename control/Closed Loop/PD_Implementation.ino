#include <Servo.h>

# define Number_of_IMU 6
#define Number_of_SERVO (Number_of_IMU-1)

int desired_angle[Number_of_IMU];
int theta[Number_of_IMU];

//Setting up Servo Instances
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;  

void write_angle()                          //For assigning the values of desired angle and passing it to the PID controller
{
  for(int k=0;k<Number_of_SERVO;k++)
  {
    desired_angle[k]=15;                      //Here we'll have to give the snake equation function
  }
  PD_inner();
}

void PD_inner()   //Passing the desired angles to the function
{             
  int kp= 0.9,kd= 1.3,PID;
  int error,previous_error,difference_error;
  
      error=0;
      while(error>5)
      {
        previous_error  = error;
        error = (desired_angle[0]-theta[1]);          //Theta is the angle given by the IMU
        difference_error  = error-previous_error;
        PID = (kp*error)  + (kd*difference_error);
        servo1.write(desired_angle[0]+PID);        
      }
      error=0;
      while(error>5)
      {
        previous_error  = error;
        error = (desired_angle[1]-theta[2]);          //Theta is the angle given by the IMU
        difference_error  = error-previous_error;
        PID = (kp*error)  + (kd*difference_error);
        servo2.write(desired_angle[1]+PID);        
      }      
      error=0;
      while(error>5)
      {
        previous_error  = error;
        error = (desired_angle[2]-theta[3]);          //Theta is the angle given by the IMU
        difference_error  = error-previous_error;
        PID = (kp*error)  + (kd*difference_error);
        servo3.write(desired_angle[2]+PID);        
      }      
      error=0;
      while(error>5)
      {
        previous_error  = error;
        error = (desired_angle[3]-theta[4]);          //Theta is the angle given by the IMU
        difference_error  = error-previous_error;
        PID = (kp*error)  + (kd*difference_error);
        servo4.write(desired_angle[3]+PID);        
      }
      error=0;
      while(error>5)
      {
        previous_error  = error;
        error = (desired_angle[4]-theta[5]);          //Theta is the angle given by the IMU
        difference_error  = error-previous_error;
        PID = (kp*error)  + (kd*difference_error);
        servo5.write(desired_angle[4]+PID);        
      }
            error=0;
      while(error>5)
      {
        previous_error  = error;
        error = (desired_angle[5]-theta[6]);          //Theta is the angle given by the IMU
        difference_error  = error-previous_error;
        PID = (kp*error)  + (kd*difference_error);
        servo6.write(desired_angle[5]+PID);        
      }
  
}

void setup() 
{
  servo1.attach(3);
  servo2.attach(10);
  servo3.attach(9);
  servo4.attach(5);
  servo5.attach(2);
  servo6.attach(8);
  
//Put snake in starting position
   servo1.write(90);
   servo2.write(90);
   servo3.write(90);
   servo4.write(90);
   servo5.write(90);
   servo6.write(90-13); 
   
}

void loop()
{  
  write_angle();
}
