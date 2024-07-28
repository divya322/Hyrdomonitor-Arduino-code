/* Sensors: 

1. Heart rate  pin  -  I2C
2. Gyro  pin - 
3. Microphone  pin  - 8
4. Temp    pin  - 7
5. Pressure sensor -  2 , 3


*/

#include <Wire.h>

//sd card
#include <SD.h>
#include <SPI.h>
File myFile;
int pinCS = 53; 
//sd card end

//df robot heart
#include <DFRobot_BloodOxygen_S.h>  //dfrobot heart rate sensor
#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked
#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x57
  DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);
#else
/* ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 * ---------------------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(4, 5);
DFRobot_BloodOxygen_S_SoftWareUart MAX30102(&mySerial, 9600);
#else
DFRobot_BloodOxygen_S_HardWareUart MAX30102(&Serial1, 9600); 
#endif
#endif
//df robot heart end

//pressure sensor
#include "HX711.h"

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 4;
const int LOADCELL_SCK_PIN = 3;
long rd1 = 0;
long rd =0;
HX711 scale;
//pressure sensor end

//screen
#include <Adafruit_GFX.h>        //OLED libraries
#include <Adafruit_SSD1306.h>    //OLED libraries
#define SCREEN_WIDTH  128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//screen end

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)

//display logo
static const unsigned char PROGMEM logo2_bmp[] =
{ 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,              //Logo2 and Logo3 are two bmp pictures that display on the OLED if called
0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,  };

static const unsigned char PROGMEM logo3_bmp[] =
{ 0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00  };
//display logo end

//mic
int sensorData;
unsigned long lastEvent = 0;
String snr="";
//mic end

//gyro
int ADXL345 = 0x53, a11=0; // The ADXL345 sensor I2C address
float x=0, y=0, z=0;  // Outputs
float x1=x,y1=y,z1=z;
int chck_acc();
String inac="Inactivity detected";
String ac="activity detected";
String curr="";
//gyro end

//temp
#define ntc_pin A0         // Pin,to which the voltage divider is connected
#define vd_power_pin 2        // 5V for the voltage divider
#define nominal_resistance 10000       //Nominal resistance at 25⁰C
#define nominal_temeprature 25   // temperature for nominal resistance (almost always 25⁰ C)
#define samplingrate 5    // Number of samples
#define beta 3950  // The beta coefficient or the B value of the thermistor (usually 3000-4000) check the datasheet for the accurate value.
#define Rref 10000   //Value of  resistor used for the voltage divider
int samples = 0;   //array to store the samples

//temp end


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(); // Initiate the Wire library
  Wire.beginTransmission(ADXL345); // Start communicating with the device 
  Wire.write(0x2D); // Access/ talk to POWER_CTL Register - 0x2D
  Wire.write(8); // (8dec -> 0000 1000 binary) Bit D3 High for measuring enable
  Wire.endTransmission();  //accel

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);  //pressure sensor

  pinMode(vd_power_pin, OUTPUT);  //Temp
  pinMode(A8, INPUT); //microphone

  //Start the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c); 
  display.display();
  display.clearDisplay();
  //End the OLED 

  //heart
  while (false == MAX30102.begin())  
  {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect();
  //heart end

  //sd card 
  pinMode(pinCS, OUTPUT);
  // Create/Open file 
  // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
  //sd card end
  
}
void loop() {

  //Temp start
  uint8_t i;
  float average;
  samples = 0;
  // take voltage readings from the voltage divider
  digitalWrite(vd_power_pin, HIGH);
  for (i = 0; i < samplingrate; i++) {
    samples += analogRead(ntc_pin);
    delay(10);
  }
  digitalWrite(vd_power_pin, LOW);
  average = 0;
  average = samples / samplingrate;
  Serial.println("\n \n");
  //Serial.print("ADC readings ");
  //Serial.println(average);
  // Calculate NTC resistance
  average = 1023 / average - 1;
  average = Rref / average;
  //Serial.print("Thermistor resistance ");
  //Serial.println(average);
  float temperature;
  temperature = average / nominal_resistance;     // (R/Ro)
  temperature = log(temperature);                  // ln(R/Ro)
  temperature /= beta;                   // 1/B * ln(R/Ro)
  temperature += 1.0 / (nominal_temeprature + 273.15); // + (1/To)
  temperature = 1.0 / temperature;                 // Invert
  temperature -= 273.15;                         // convert absolute temp to C
  Serial.print("Temperature ");
  Serial.print(temperature);
  Serial.print(" *C");
  Serial.print("ADC readings ");
  Serial.println(average);
  //Temp End

  //mic
  sensorData = analogRead(8);
  Serial.print("mic: ");
  Serial.println(sensorData);
  /*filtered
	// If pin goes LOW, sound is detected
	if (sensorData == LOW) {
		// If 25ms have passed since last LOW state, it means that
		// the clap is detected and not due to any spurious sounds
		if (millis() - lastEvent > 25) {
			Serial.println("Snoring detected!");
      snr = "Snoring detected!";
		}
    lastEvent = millis();
		// Remember when last event happened
	}
  if (millis() - lastEvent > 100){
    snr = "No Snoring detected!";
  }
  filtered*/
  snr=sensorData;
  //mic end

  //pressure sensor
    if (scale.is_ready()) {
    rd = scale.read();
    Serial.print("pressure: ");
    Serial.print(rd);
    Serial.print("   ");
    Serial.println(rd-rd1);
    rd1 = rd;
  } else {
    Serial.println("HX711 not found.");
  }
  //pressure sensor end

  //heart
  MAX30102.getHeartbeatSPO2();
  Serial.print("SPO2 is : ");
  Serial.print(MAX30102._sHeartbeatSPO2.SPO2);
  Serial.println("%");
  Serial.print("heart rate is : ");
  Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat);
  Serial.println("Times/min");
  //Serial.print("Temperature value of the board is : ");
  //Serial.print(MAX30102.getTemperature_C());
  //Serial.println(" ℃");
  //heart end

  //gyro
  chck_acc();
  x1=x;
  y1=y;
  z1=z;
  pri(a11);
  //gyro end

  //sd card
  myFile = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Writing to file...");
    // Write to file
    myFile.println(String(MAX30102._sHeartbeatSPO2.Heartbeat)+"bpm , "+curr+" , "+"mic: "+snr+" , "+temperature+" Celcius, pr d: "+rd+" , pr diff: "+rd1);
    myFile.close(); // close the file
    Serial.println("Done.");
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening test.txt");
  }
  display.clearDisplay();
  //sd card end
  delay(1000);
}


int chck_acc(){
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32); // Start with register 0x32 (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  x = ( Wire.read()| Wire.read() << 8); // X-axis value
  x = x/256; //For a range of +-2g, we need to divide the raw values by 256, according to the datasheet
  y = ( Wire.read()| Wire.read() << 8); // Y-axis value
  y = y/256;
  z = ( Wire.read()| Wire.read() << 8); // Z-axis value
  z = z/256;
  Serial.print("Xa= ");
  Serial.print(x);
  Serial.print("   Ya= ");
  Serial.print(y);
  Serial.print("   Za= ");
  Serial.println(z);
  if (abs(x1-x)>0.05 || abs(y1-y)>0.05 || abs(z1-z)>0.05){
    a11=1;
    Serial.println("x1-x= "+String(abs(x1-x))+"   y1-y= "+String(abs(y1-y))+"   z1-z= "+String(abs(z1-z)));
  }
  else
    a11=0;
}

void pri(int a11){
  display.setTextSize(1);                    
  display.setTextColor(WHITE);             
  display.setCursor(5,35);         
  if (a11==0){
      display.println(inac); 
      Serial.println(inac);
      curr=inac;
  }
  else{
    display.println(ac); 
    Serial.println(ac);
    curr=ac;
  }
  display.display();
}
