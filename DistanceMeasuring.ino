//By using I2Cdev and MPU6050 libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>

MPU6050 accelgyro;

SoftwareSerial printer(0, 1);

int16_t ax, ay, az;
int16_t gx, gy, gz;

double correctionay, correctionaz, correctiongx;
double prevay=-600, prevaz=16250, prevgx=202;
double curay, curaz, curgx;

double basefare = 50.00 ; double unitfare = 40.00 ;

double smoothratio;

double anglex = 0;
double dt = 0.001;
double radianconv = 0.017;
double speedy = 0;
double dist = 0;
double totalfare = 0;

int addr = 0;

const int buttonCalib = 7;
int buttonCalibState;
int buttonCalibLastState = LOW;

const int buttonHire = 6;
int buttonHireState;
int buttonHireLastState = LOW;

const int buttonPrint = 8;
int buttonPrintState;
int buttonPrintLastState = LOW;

long lastDebounceTime = 0;  
long debounceDelay = 50;  

int onhire = LOW ;

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup(){
  
  Serial.begin(9600);
  accelgyro.initialize();
  lcd.begin(16, 2); 
  pinMode(buttonCalib, INPUT);
  pinMode(buttonHire, INPUT);
  printer.begin(4800);
    
}




void loop(){
  calculate();
   if(onhire == HIGH){
     speedy = 0;
     dist = 0; 
     integrator();
   }
   else{
     printRequest();
   }
   
}


void printRequest(){
  
  int reading = digitalRead(buttonPrint);

  if (reading != buttonPrintLastState) {
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonPrintState) {
      buttonPrintState = reading;

      
      if (buttonPrintState == HIGH) {
        printBill();
      }
    }
  }
  
  buttonCalibLastState = reading;
  
}

void printBill(){
  printer.print("Distance: "); printer.println(dist);
  printer.print("Fare: "); printer.println(totalfare);
}



void integrator(){
  
   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   //Serial.print("a/g:\t");
   //Serial.print(ax); Serial.print("\t");
   
   Serial.print(ay); Serial.print("\t");
   double smoothenay = getSmoothen(prevay,ay);
   Serial.print(smoothenay); Serial.print("\t");
   prevay = smoothenay;
   
   Serial.print(az); Serial.print("\t");
   double smoothenaz = getSmoothen(prevaz,az);
   Serial.print(smoothenaz); Serial.print("\t");
   prevaz = smoothenaz;
   
   Serial.print(gx); Serial.print("\t");
   double smoothengx = getSmoothen(prevgx,gx);
   Serial.print(smoothengx); Serial.print("\t");
   prevgx = smoothengx;
      
   //Serial.print(gy); Serial.print("\t");
   //Serial.println(gz);
   
   anglex += (gx+202) * dt ;
   //double gravitycomp = 16400 * cos(anglex * radianconv);
   double zgravity = (smoothenaz * 9.806) / 16250 ;
   double temp = 96.1576 - zgravity*zgravity;
   if(temp < 0){
     temp = 0 ;
   }
   
   double gravitycomp = sqrt(temp);
   double ytotal = ((ay+220) * 9.806) / 16400 ; 
   double realaccely ;   
   if(anglex > 0){
     realaccely = ytotal - gravitycomp ;
   }
   else{
     realaccely = ytotal + gravitycomp ;  
   }
   
   if(realaccely < 1.5 && realaccely > -1.5 ){
     realaccely = 0;
   }
    
   Serial.print(anglex); Serial.print("\t");
   Serial.print(realaccely); Serial.print("\t");
   
   realaccely *= 3.6;
   speedy += realaccely * dt ;
   
     
   if (speedy > 0){
     dist += speedy/3600 * dt;
   }
   else{
     dist -= speedy/3600 * dt;
   }
   
   Serial.println(speedy);
   //***
   
     
   lcd.setCursor(0, 0);
   lcd.print(speedy/100);
   
   lcd.setCursor(9, 0);
   lcd.print(dist);
   
   lcd.setCursor(0, 1);
   lcd.print(millis()/1000);
   
   lcd.setCursor(9, 1);
   lcd.print(getFare(dist));
      
   delay(1000*dt);

} 
   


void setCorrection(){

  double aysum = 0;
  double azsum = 0;
  double gxsum = 0;

  int samps = 10;
  
  for(int i=0; i < samps ; i++){
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    aysum += ay;
    azsum += az;
    gxsum += gx;  
  }
  
  correctionay = aysum/samps ;
  correctionaz = azsum/samps ;
  correctiongx = gxsum/samps ;
  
  
  EEPROM.write(addr,correctionay);
  EEPROM.write(addr+1,correctionaz);
  EEPROM.write(addr+2,correctiongx);
}

void getCorrection(){
  correctionay = EEPROM.read(addr);
  correctionaz = EEPROM.read(addr);
  correctiongx = EEPROM.read(addr);
}

double getSmoothen(double prev, double cur){
  return (prev * 0.99 + cur * 0.01);
}

double getFare(double dist){
  if(dist < 1){
    totalfare = basefare;
    return totalfare;
  }
  else{
    totalfare = basefare + (dist - 1) * unitfare ;
    return totalfare;
  }
}

void calculate(){
  
  int reading = digitalRead(buttonHire);

  if (reading != buttonHireLastState) {
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonHireState) {
      buttonHireState = reading;      
      if (buttonHireState == HIGH) {
        if(onhire == HIGH ){ onhire = LOW; }
        else { onhire = HIGH; }
      }
    }
  }
  
  buttonHireLastState = reading;
  
}

void caliberate(){
  
  int reading = digitalRead(buttonCalib);

  if (reading != buttonCalibLastState) {
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonCalibState) {
      buttonCalibState = reading;

      
      if (buttonCalibState == HIGH) {
        setCorrection();
        delay(10);
        getCorrection();
      }
    }
  }
  
  buttonCalibLastState = reading;
  
}
