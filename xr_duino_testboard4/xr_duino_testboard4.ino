/*
xr_duino Testboard für Arduino at Breadboard, der eigene Arduino
xrcontrol.de, noc0815@gmail.com /12-2013

Für ATMEGA 328P

Software zum testen des selbst gebrannten Arduino Bootloader

Hardware: 

Leds an den AVR_Pins!(Pin Mapping beachten):
2,3,12-19,24-26

BS250 steuert an AVR!Pin 11 LCD-Backlight

Poti 10k an AVR!Pin 23

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
  lcd.print("-XrDuino-");  
  lcd.setCursor ( 0, 1 );        // go to the next line
  lcd.print ("XrControl");
  
  Serial.println("XrDuino Testboard");
  Serial.println("xrcontrol.de");
  Serial.println("noc0815@gmail.com");
  Serial.end();
  
  delay ( 1500 );
  lcd.clear();
  lcd.home();
  lcd.print("Serial:");
  lcd.setCursor(0,1);
  lcd.print("9600 baud");
  delay(1500);
  lcd.clear();
  lcd.home();
  lcd.print("SEC:");
}

void loop()
{
  if (millis()-ledspeed_time>aktledspeed){  //Zeitsteuerung Lauflicht
   checkleds();
  } 
  if (millis()-secflag_time>1000){          //Sekundenzähler und com bedienen
    checktime();
  }
  if (millis()-backlight_time>50){          //LCD Backlight steuern
    checkbacklight();
  }
  if (millis()-key_time>100){               // Tasten und Poti abfragen
    checkkey();
  } 
}//void loop

void checkkey(){
  
  if(digitalRead(key1_pin)==LOW){        // Taster 1 gedrückt?
    lcd.setCursor(0,1);
    lcd.print("T.1/A#");
    ledmode=1;
    lcd.home();
    lcd.print("AUT");
  }//if digitalread
  else if (digitalRead(key2_pin)==LOW){  // Taster 2 gedrückt?
    lcd.setCursor(0,1);
    lcd.print("T.2/A#");
    ledmode=0;
    lcd.home();
    lcd.print("MAN");
  }//else if
  else if (digitalRead(key3_pin)==LOW){  // Taster 3 gedrückt?
    lcd.setCursor(0,1);
    lcd.print("T.3/A#");
  }//else if
  else{
    lcd.setCursor(0,1);
    lcd.print("---/A#");                // Keine Taste betätigt
  }//else
  poti_value=analogRead(poti_pin)/10;   // Potiwert einlesen 
  lcd.setCursor(6,1);
  lcd.print(poti_value);                // und auf LCD ausgeben
  lcd.print(" ");
  if(ledmode==0){
    aktledspeed=poti_value;
  }//if ledmode
}//void checkkey

void checkleds(){ //ledlauflicht auf/ab
  digitalWrite(myLeds[aktled],LOW);
  if(led_direction==1){
    aktled++;                //nächste Led aufwärts
    if(aktled>12){
      led_direction=0;
      aktled=11;
    }//if aktled
  } // if led_direction  
  else{
    aktled--;                 //nächste Led abwärts
    if(aktled<0){
      led_direction=1;
      aktled=1;
      if(ledmode==1){      //nur im automode ledspeed ändern
        checkledspeed();
      }//if ledmode
    } //if aktled
  } //else
  digitalWrite(myLeds[aktled],HIGH);
  ledspeed_time=millis();  
}//void checkleds

void checkledspeed(){          //Lauflicht Geschwindigkeit
  aktledspeed=aktledspeed+5;
  if (aktledspeed>80){
    aktledspeed=10;
  }//if aktledspeed

} //void checkledspeed

void checktime(){  //Ausgabe der abgelaufenen Sekunden
  
  lcd.setCursor (4,0);
  lcd.print(millis()/1000);
  Serial.begin(9600);         //Übertrage Sekunden und Potiwert auf com
  Serial.print("Sec.: ");
  Serial.print(millis()/1000);
  Serial.print(" / Poti: #");
  Serial.println(poti_value);
  Serial.end();
  secflag_time=millis();
}

void checkbacklight(){    //Auf- und Ab Dimmen des Backlights
  if (backlight_direction==1){
    backlight_value=backlight_value+5;    //aufdimmen
    if (backlight_value>200){
      backlight_direction=0; 
      backlight_value=195;
    }//if backlight_value
  }//if backlight_direction
  else{
    backlight_value=backlight_value-5;    //abdimmen
    if(backlight_value<0){
      backlight_direction=1;
      backlight_value=5;
    }//if backlight_value
  }//else
  analogWrite(backlight_pin,backlight_value);
  backlight_time=millis();  
} //void checkbacklight
  
