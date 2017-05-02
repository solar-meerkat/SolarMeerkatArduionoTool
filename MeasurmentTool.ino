//current, dht,pressure,BT, Volt,axel,compas

#include <Wire.h>
//BT and led
enum { LED_PIN = 13 };
enum LedState { LED_ON, LED_OFF, LED_BLINK };
LedState led_state;

//DHT11
#include "DHT.h"
#define DHTPIN 2     // what digital pin we're connected to
#define DHTTYPE DHT11   // DHT 11
DHT dht(DHTPIN, DHTTYPE);

//current
#include <Adafruit_INA219.h> // You will need to download this library
Adafruit_INA219 sensor219; // Declare and instance of INA219

//pressure
#include <SFE_BMP180.h>
SFE_BMP180 pressure;

//Gyro
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

void setup()
{
  Serial.begin(9600);
  Serial.println("Solar-2 test!");
//BT and LED
  led_state = LED_OFF;
  pinMode(LED_PIN, OUTPUT);
//pinMode(3,OUTPUT);
//DHT
  dht.begin();
//current
  sensor219.begin();
//Gyro
  Wire.begin();
  setupMPU();
//pressure
  if (pressure.begin())
    Serial.println(/*"BMP180 init success"*/);
  else
  {
    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
//  

}

//=============================================================================  S T A R T
void loop()
{  
   Serial.println();
//============================================================================= DHT begin
  // Wait a few seconds between measurements.
  delay(1000);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);

  Serial.println("Humidity: ");
  Serial.println(h);
/* Serial.print(" %\t"); */
  Serial.println("hTemp: ");
  Serial.println(t);
//  Serial.print(" C ");
/*  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F"); */
//============================================================================ DHT end
//============================================================================ pressure begin
  char status;
  double T,P;
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.println("pTemp: ");
      Serial.println(T,2);
//      Serial.print(" deg C, ");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.println("absolute pressure: ");
          Serial.println(P,2);
//          Serial.println(" mb, ");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  delay(3000);  // Pause for 5 seconds.
//============================================================================ pressure end
//============================================================================ CURRENT begin
  float busVoltage = 0;
  float current = 0; // Measure in milli amps
  float power = 0;

  busVoltage = sensor219.getBusVoltage_V();
  current = sensor219.getCurrent_mA();
  power = busVoltage * (current/1000); // Calculate the Power
  
  
  Serial.println("Bus Voltage:   "); 
  Serial.println(busVoltage); 
//  Serial.println(" V");  
  
  Serial.println("Current:       "); 
  Serial.println(current); 
//  Serial.println(" mA");
  
/*  Serial.print("Power:         "); 
  Serial.print(power); 
  Serial.println(" W");  */


  delay(1000);
//============================================================================ CURRENT end
//============================================================================ Voltage begin
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println("AnalogVoltage:         ");   
  Serial.println(voltage);
//============================================================================ Voltage end
//============================================================================ Gyro begin
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  delay(100);
//============================================================================ Gyro end  
//============================================================================ bluetooth begin
/*  if (Serial.available())
  {
    char command = Serial.read();
    
    switch (command)
    {
      case '1':
        led_state = LED_ON; 
        Serial.println("Led On Cool");
        break;
        
      case '0': 
        led_state = LED_OFF; 
        Serial.println("Led Off Fall");
        break;
      case '*': 
        led_state = LED_BLINK;
        Serial.println("Led Blink ");
        break;
      
      default:
      {
        for (int i = 0; i < 5; ++i)
        {
          
          digitalWrite(LED_PIN, HIGH);
          delay(50);
          digitalWrite(LED_PIN, LOW);
          delay(50);
        }
        Serial.println("Fail ....");
      }
    }
  }
  
  switch (led_state)
  {
    case LED_ON: digitalWrite(LED_PIN, HIGH); break;
    case LED_OFF: digitalWrite(LED_PIN, LOW); break;
    
    case LED_BLINK:
    {
      static unsigned long start_millis = 0;
      
      if (millis() - start_millis >= 300)
      {
        start_millis = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      }
    }
  } */
//================================================================================== bluetooth end
}

//********************************************************************************************************   Gyro functions    ***************************************
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}
