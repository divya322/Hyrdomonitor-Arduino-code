//Includes the Arduino Stepper Library
#include <Stepper.h>

#include <dht.h>        // Include library
#define outPin 6        // Defines pin number to which the dht sensor is connected
dht DHT;                // Creates a DHT object

const int AirValue = 520;   //you need to replace this value with Value_1
const int WaterValue = 260;  //you need to replace this value with Value_2
int intervals = (AirValue - WaterValue)/3;
int soilMoistureValue = 0;

#define WATER_PIN A2
int water = 0;

const int stepsPerRevolution = 2038;    // Defines the number of steps per rotation
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11); // Creates an instance of stepper class, Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence

const int ldrPin = A0;
String flap="open";

String url="";

void setup() {
    // Nothing to do (Stepper Library sets pins as outputs)
    Serial.begin(9600);
    pinMode(ldrPin, INPUT);
    pinMode(51, OUTPUT);
    pinMode(52, OUTPUT);
    pinMode(50, OUTPUT);
    digitalWrite(50,LOW);
    digitalWrite(51,LOW);
    digitalWrite(52,LOW);
    //myStepper.setSpeed(10);
    //myStepper.step(430);
}

void loop() { 
  url="";
  int ldr = analogRead(ldrPin);
  if (ldr >= 900) {
    //Serial.print("Its daytime Time: ");
    //Serial.println(ldr);
    if (flap=="open"){
    myStepper.setSpeed(10);
    myStepper.step(1000);
    digitalWrite(50,HIGH);
    digitalWrite(51,HIGH);
    digitalWrite(52,HIGH);
    flap="close";
    delay(2000);
    }
  } 
  else {
    //Serial.print("Its nighttime, Turn off the LED : ");
    if (flap=="close"){
    myStepper.setSpeed(10);
    myStepper.step(-630);
    digitalWrite(50,LOW);
    digitalWrite(51,LOW);
    digitalWrite(52,LOW);
    flap="open";
    delay(2000);
    }
    //Serial.println(ldr);
    }
    //Serial.print("Flap: ");
    //Serial.println(flap);

    int readData = DHT.read11(outPin);
    float t = DHT.temperature;        // Read temperature
    float h = DHT.humidity;           // Read humidity
    /*Serial.print("Temperature = ");
    Serial.print(t);
    Serial.print("°C | ");
    Serial.print((t*9.0)/5.0+32.0);        // Convert celsius to fahrenheit
    Serial.println("°F ");
    Serial.print("Humidity = ");
    Serial.print(h);
    Serial.println("% ");
    Serial.println("");*/

    soilMoistureValue = analogRead(A1);  //put Sensor insert into soil
    //Serial.print("soil Moisture Value = ");
    //Serial.println(soilMoistureValue);
    int soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
    //Serial.println(soilmoisturepercent);
    if(soilMoistureValue > WaterValue && soilMoistureValue < (WaterValue + intervals))
    {
      //Serial.println("Very Wet");
    }
    else if(soilMoistureValue > (WaterValue + intervals) && soilMoistureValue < (AirValue - intervals))
    {
      //Serial.println("Wet");
    }
    else if(soilMoistureValue < AirValue && soilMoistureValue > (AirValue - intervals))
    {
      //Serial.println("Dry");
    }    
    //Serial.print("\n");

    water = analogRead(WATER_PIN);
    //Serial.print("water Value = ");
    //Serial.println(water);
    int nitro=150,phos=150,pott=150;
    url=String(water)+"+"+String(ldr)+"+"+String(soilMoistureValue)+"+"+String(soilMoistureValue)+"+"+String(t)+"+"+String(h)+"+"+flap+"+"+String(nitro)+"+"+String(phos)+"+"+String(pott);
    Serial.print(url);
    Serial.print("\n");
    delay(2000);
}
