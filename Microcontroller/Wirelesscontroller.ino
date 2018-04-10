/*
* Arduino Wireless Communication Tutorial
*     Example 1 - Transmitter Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* 
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include<avr/io.h>
#include<stdlib.h>
RF24 radio(7, 8); // CE, CSN
int buttonpress=0;
int k=0;
const byte address[6] = "00001";
void setup() {
  //DDRD&=~((1<<2)|(1<<3));
  DDRD=0X00;
    sei();//enable interrupts
 EIMSK|=(1<<INT0)|(1<<INT1);//setting pin pd2 for accessing the interrupts
  EICRA|=(1<<ISC00)|(1<<ISC01)|(1<<ISC11)|(1<<ISC10);//RISING EDGE
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  adc_init();

}
void adc_init()
{
  //Vcc is reference voltage, left shift adc result
   ADMUX=0b01100000;
  //Enable adc, prescaler 128
  ADCSRA |= ((1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
}



int adc_read()
{    ADMUX=0b01100000;
    ADCSRA|=(1<<ADSC);
    while(!(ADCSRA&(1<<ADIF)));
    ADCSRA|=(1<<ADIF);
    return ADCH;
}
int adc_read1()
{
    ADMUX=0b01100001;
    ADCSRA|=(1<<ADSC);
    while(!(ADCSRA&(1<<ADIF)));
    ADCSRA|=(1<<ADIF);
    return ADCH;
}
 int  ch,ch1;
 char a;
void loop() {
  ch = adc_read();
    ch1 = adc_read1();
     // Serial.println(ch);
     if (ch>115&&ch<240&&ch1<10)
    a='0';
    
   else if (ch>240&&ch1<10)
     a='1';
   else if (ch>240&&ch1>115&&ch1<140)
    a='2';
   else if (ch>240&&ch1>240)
    a='3';
  else if(ch>115&&ch<140&&ch1>240)
    a='4';
   else if (ch<10&&ch1>240)
    a='5';
   else if (ch<10&&ch1>115&&ch1<140)
    a='6';
   else if (ch<10&&ch1<10)
   a='7';
  
radio.write(&a, sizeof(a));
 
}
ISR(INT0_vect)
{PORTB^=(1<<PINB5);
buttonpress--;
if(buttonpress ==-1)
buttonpress=4;
 _delay_ms(10);//To prevent the mcu from reading the data in bouncing region
}
 ISR(INT1_vect)
{PORTB^=(1<<PINB5);
 buttonpress++;

 if(buttonpress==4)
 buttonpress=0;
 _delay_ms(10);//To prevent the mcu from reading the data in bouncing region
}
 
  
 
