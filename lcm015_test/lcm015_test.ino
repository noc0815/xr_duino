/*
xr_duino Testboard für Arduino at Breadboard, der eigene Arduino
xrcontrol.de, noc0815@gmail.com /12-2013

Für ATMEGA 328P

Software zum testen des selbst gebrannten Arduino Bootloader

Hardware: 

LCM015 LCD an I2C
I2C LCD an AVR!PIN 27+28

FTDI Modul(RX/TX/DTR) an AVR!Pin 3,2,1

Die Software ist Public Domain(non profit)

*/



#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

unsigned long backlight_time;
unsigned long ledspeed_time;
unsigned long secflag_time;
unsigned long key_time;
int led_direction=1;
int aktledspeed=1;
int backlight_direction=1;
int backlight_value=10;
int aktled=0;
int myLeds[] = {A3,A2,A1,13,12,11,6,10,1,0,9,8,7};   //led pin nr. einlesen
int poti_value=0;
int ledmode=1;

#define backlight_pin 5
#define key1_pin 4
#define key2_pin 2
#define key3_pin 3
#define poti_pin A0

LiquidCrystal_I2C lcd(0x27);  // Set the LCD I2C address

//LiquidCrystal_I2C lcd(0x27, backlight_pin, POSITIVE);  // Set the LCD I2C address



void setup()
{
  Serial.begin(9600); 
  // Switch on the backlight
  secflag_time=millis();
  backlight_time=millis();
  ledspeed_time=millis();
  key_time=millis();
  pinMode(key1_pin,INPUT_PULLUP);    //3 tasten auf eingang
  pinMode(key2_pin,INPUT_PULLUP);    //int. pullups ein
  pinMode(key3_pin,INPUT_PULLUP);
  
  for (int i=0; i <= 12; i++){      //alle led pins auf ausgang und ausschalten
      pinMode(myLeds[i],OUTPUT);
      digitalWrite(myLeds[i],LOW);
  }// for 
  
  pinMode ( backlight_pin, OUTPUT );
  analogWrite ( backlight_pin, backlight_value );
  
  lcd.begin(10,2);               // initialize the lcd 

  lcd.home ();                   // go home
  lcd.print("LCM015_I2C");  
  lcd.setCursor ( 0, 1 );        // go to the next line
  lcd.print ("XrControl");
  
  
}

void loop()
{
}//void loop


