#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include<avr/interrupt.h>

/*
 * Serpentine lateral plane:
 * a=0.6*pi
 * b=2*pi
 * c=0
 * 
 *  a=0.75*pi;
    b=3.5*pi;
    c=-0.2*pi;
    num_segments = 5;

          a=1.5*pi;
          b=3*pi;
          c=0*pi;


    
 */

 /*
  * Rolling gait:
  * a=0.9*pi
  * b=1*pi
  * c=0
  * n=8
  * phase diff += 0.1
  * add pi to switch direction
  */
 
// Define servo objects for the snake segments
Servo s1; 
Servo s2;
Servo s3;
Servo s4; 
Servo s5;
Servo s6;
Servo s7;
int direc =48;
const float pi = 3.141593;
int counter = 0; 
int frequency = 1; 
int startPause = 2000; 
int data=52;
int previousdata=52;
float a = 0.9*pi;                                   //Ampli Coeff
float b = 1*pi;                                     //Curve Coeff      
float c = 0;                                        //Turn Coeff
float num_segments = 8;
float gamma=-c/num_segments;
float beta=b/num_segments;
float alpha=a*abs(sin(beta/2));
unsigned long previousMillis = 0;
long interval = 4;
void setup() 
{
  Serial.begin(9600);

  // Attach segments to pins  
   s1.attach(3); //blue left - 1st servo
   s2.attach(10); //black right - 2nd servo
   s3.attach(9); //violet right - 3rd servo 
   s4.attach(5); //black left - 4th servo 
   s5.attach(2); //white left - 5th servo 
   s6.attach(8); //blue right - 6th servo 
   s7.attach(4); //brown left - 7th servo 
 
//Put snake in starting position
   s1.write(90);
   s2.write(90);
   s3.write(90);
   s4.write(90);
   s5.write(90);
   s6.write(90-13); 
   s7.write(90-7); 
   delay(startPause);
   previousMillis = millis();
}
void loop() 
{
   if(Serial.available() > 0)      // Send data only when you receive data:
   {
      data = Serial.read();        //Read the incoming data & store into data
      Serial.print(data);          //Print Value inside data in Serial monitor  
      if(data != 52 && data !=53)
      {
        direc = data; 
        data = previousdata;
        Serial.println("direction added");
      }
   }
  if(data==52)
  {/*
    Serial.println("Helical on");
    //Helical gait
         if(direc ==48)
        {
          Serial.println("heli straight");
          a=0.8*pi;
          b=2*pi;
          c=0;
          num_segments = 8;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("helical");
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              //Serial.println((alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              s2.write(90 + (alpha*sin(frequency*counter*pi/180+1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180+2*beta)+gamma)*180/pi);
              s4.write(90 + (alpha*sin(frequency*counter*pi/180+3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180+4*beta)+gamma)*180/pi);
              s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180+6*beta)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc ==49)
        {
          Serial.println("heli right");
          a=0.8*pi;
          b=2*pi;
          c=0;
          num_segments = 8;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("helical");
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              //Serial.println((alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              s2.write(90 + (alpha*sin(frequency*counter*pi/180+1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180+2*beta)+gamma)*180/pi);
              s4.write(90 + (alpha*sin(frequency*counter*pi/180+3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180+4*beta)+gamma)*180/pi);
              s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180+6*beta)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc ==50)
        {
          Serial.println("heli back");
          a=0.8*pi;
          b=2*pi;
          c=0;
          num_segments = 8;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("helical");
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              //Serial.println((alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              s2.write(90 + (alpha*sin(frequency*counter*pi/180+1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180+2*beta)+gamma)*180/pi);
              s4.write(90 + (alpha*sin(frequency*counter*pi/180+3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180+4*beta)+gamma)*180/pi);
              s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180+6*beta)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc ==51)
        {
          Serial.println("heli left");
          a=0.8*pi;
          b=2*pi;
          c=0;
          num_segments = 8;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("helical");
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              //Serial.println((alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              s2.write(90 + (alpha*sin(frequency*counter*pi/180+1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180+2*beta)+gamma)*180/pi);
              s4.write(90 + (alpha*sin(frequency*counter*pi/180+3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180+4*beta)+gamma)*180/pi);
              s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180+6*beta)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
    previousdata = data;
    */
        //Serpentine gait
         if(direc == 48)
        {
          //Serial.println("hell");
          a=0.6*pi;
          b=2*pi;
          c=0*pi;
          num_segments = 5;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          interval = 3;
          //Serial.println("Serpentine forward");
            // Serpentine motion
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180-0*beta)+gamma)*180/pi);
              //Serial.println(90 + (alpha*sin(frequency*counter*pi/180-1*beta)+gamma)*180/pi);
              //s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180-2*beta)+gamma)*180/pi);
              //s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180-4*beta)+gamma)*180/pi);
              //s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180-6*beta)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc == 49)
        {
          a=0.6*pi;
          b=2*pi;
          c=0.2*pi;
          num_segments = 5;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("Serpentine right");
          
            // Serpentine motion right
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180-1*beta+pi)+gamma)*180/pi);
              //Serial.println(90 + (alpha*sin(frequency*counter*pi/180-1*beta)+gamma)*180/pi);
              //s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180-2*beta+pi)+gamma)*180/pi);
              //s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180-3*beta+pi)+gamma)*180/pi);
              //s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180-4*beta+pi)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc == 50)
        {
          a=0.6*pi;
          b=2*pi;
          c=-0.2*pi;
          num_segments = 5;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("Serpentine left");
          
            // Serpentine motion left
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180-1*beta+pi)+gamma)*180/pi);
              //Serial.println(90 + (alpha*sin(frequency*counter*pi/180-1*beta)+gamma)*180/pi);
              //s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180-2*beta+pi)+gamma)*180/pi);
              //s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180-3*beta+pi)+gamma)*180/pi);
              //s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180-4*beta+pi)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc == 51)
        {
          a=0.6*pi;
          b=2*pi;
          c=0*pi;
          num_segments = 5;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("Serpentine backward");
          
            // Serpentine motion
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180-1*beta+pi)+gamma)*180/pi);
              //Serial.println(90 + (alpha*sin(frequency*counter*pi/180-1*beta)+gamma)*180/pi);
              //s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180-2*beta+pi)+gamma)*180/pi);
              //s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180-3*beta+pi)+gamma)*180/pi);
              //s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180-4*beta+pi)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        previousdata = data;
  }
  else if(data==53)
  {
    Serial.println("Rolling");
    if(direc != 49 && direc != 51)
    {
      direc = 49;
    }
    //Rolling gait
          if(direc == 49)
        {
          Serial.println("Starting to roll right");
          //Roll right
          a=0.9*pi;
          b=1.0*pi;
          c=0;
          num_segments = 8;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          interval = 7;
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval);
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180+0*beta)+gamma)*180/pi);
              s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180+0.2*beta)+gamma)*180/pi);
              s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180+0.4*beta)+gamma)*180/pi);
              s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180+0.6*beta)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
         else if(direc == 51)
        {
          Serial.println("Rollingleft");
          //Roll left
          a=0.9*pi;
          b=1*pi;
          c=0;
          num_segments = 8;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          interval = 7;
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180+0*beta + pi)+gamma)*180/pi);
              s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2 + pi)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180+0.2*beta + pi)+gamma)*180/pi);
              s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2 + pi)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180+0.4*beta+pi)+gamma)*180/pi);
              s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2+pi)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180+0.6*beta+pi)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
    previousdata = data;

   s1.write(90);
   s2.write(90);
   s3.write(90);
   s4.write(90);
   s5.write(90);
   s6.write(90-13); 
   s7.write(90-7); 
    
  }
  /*
  else if(gait==1 && flag ==0)
  {
    flag = 1;
    Serial.println("The gait is:");
    Serial.println(gait);
    Serial.println("direction:");
    Serial.println(direc);

        //Serpentine gait
         if(direc == 2)
        {
          Serial.println("hell");
          a=1.5*pi;
          b=3*pi;
          c=0*pi;
          num_segments = 4;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          interval = 3;
          //Serial.println("Serpentine forward");
            // Serpentine motion
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180-0*beta)+gamma)*180/pi);
              //Serial.println(90 + (alpha*sin(frequency*counter*pi/180-1*beta)+gamma)*180/pi);
              //s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180-2*beta)+gamma)*180/pi);
              //s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180-4*beta)+gamma)*180/pi);
              //s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180-6*beta)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc == 4)
        {
          a=0.6*pi;
          b=2*pi;
          c=0.4*pi;
          num_segments = 4;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("Serpentine right");
          
            // Serpentine motion right
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180-1*beta+pi)+gamma)*180/pi);
              //Serial.println(90 + (alpha*sin(frequency*counter*pi/180-1*beta)+gamma)*180/pi);
              //s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180-2*beta+pi)+gamma)*180/pi);
              //s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180-3*beta+pi)+gamma)*180/pi);
              //s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180-4*beta+pi)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc == 0)
        {
          a=0.6*pi;
          b=2*pi;
          c=-0.4;
          num_segments = 4;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("Serpentine left");
          
            // Serpentine motion left
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180-1*beta+pi)+gamma)*180/pi);
              //Serial.println(90 + (alpha*sin(frequency*counter*pi/180-1*beta)+gamma)*180/pi);
              //s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180-2*beta+pi)+gamma)*180/pi);
              //s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180-3*beta+pi)+gamma)*180/pi);
              //s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180-4*beta+pi)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        else if(direc == 6)
        {
          a=0.6*pi;
          b=2*pi;
          c=0;
          num_segments = 4;
          gamma=-c/num_segments;
          beta=b/num_segments;
          alpha=a*abs(sin(beta/2));
          //Serial.println("Serpentine backward");
          
            // Serpentine motion
          for(counter = 0; counter < 360;) 
          {
            unsigned long currentMillis = millis();
            if (currentMillis - previousMillis >= interval)
            {
              previousMillis = currentMillis;
              s1.write(90 + (alpha*sin(frequency*counter*pi/180-1*beta+pi)+gamma)*180/pi);
              //Serial.println(90 + (alpha*sin(frequency*counter*pi/180-1*beta)+gamma)*180/pi);
              //s2.write(90 + (alpha*sin(frequency*counter*pi/180+0.1*beta+pi/2)+gamma)*180/pi);
              s3.write(90 + (alpha*sin(frequency*counter*pi/180-2*beta+pi)+gamma)*180/pi);
              //s4.write(90 + (alpha*sin(frequency*counter*pi/180+0.3*beta+pi/2)+gamma)*180/pi);
              s5.write(90 + (alpha*sin(frequency*counter*pi/180-3*beta+pi)+gamma)*180/pi);
              //s6.write(90 - 13 + (alpha*sin(frequency*counter*pi/180+0.5*beta+pi/2)+gamma)*180/pi);
              s7.write(90 - 7 + (alpha*sin(frequency*counter*pi/180-4*beta+pi)+gamma)*180/pi);
              counter += 1;
            }       
          }
        }
        
      flag =0;
  }
  */
}
