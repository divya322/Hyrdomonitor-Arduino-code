
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

//ldr
const int ledPin = D7;
const int ldrPin = D0;
//water
int resval = 0;  // holds the value
int respin = D1; // sensor pin used
//soil
const int AirValue = 620;   //you need to replace this value with Value_1
const int WaterValue = 310;  //you need to replace this value with Value_2
int soilMoistureValue = 0;
int soilmoisturepercent=0;
//Temp, humidity
int temp = 30;
int humidity = 40;
String url = "";
const char *ssid = "akshay";  //ENTER YOUR WIFI SETTINGS
const char *password = "1234567811";
String ul;

String send_request(String ad)
{
    ul="http://192.168.73.70:47/puttingdata?sensor=";
    //Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED)
    {
        //Declare an object of class HTTPClient
        HTTPClient http;
        ul +=ad;
        Serial.print("The URL= ");
        Serial.println(ul);
        http.begin(ul);           //Specify request destination
        int httpCode = http.GET(); //Send the request
        //Check the returning code
        Serial.println(httpCode);
        if (httpCode > 0)
        {
            //Get the request response payload
            String payload = http.getString();
            //Print the response payload
            Serial.println(payload);
        }
        //Close connection
        http.end();
    }
    return "done";
}

void setup()
{
    pinMode(ledPin, OUTPUT);
    pinMode(ldrPin, INPUT);
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    
    Serial.print("Connecting.");
    // Checking Wifi connectivity
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected !");
    Serial.print("Local IP address:\t");
    Serial.println(WiFi.localIP());
  // Sending request
}

void loop()
{
  
url = "";
resval = analogRead(respin);
Serial.print("Water level: ");
Serial.println(resval);
if (resval<=100){ 
  Serial.println("Water Level: Empty"); 
  } 
/*else if (resval>100 && resval<=300){ 
  Serial.println("Water Level: Low"); 
  } 
else if (resval>300 && resval<=330){ 
  Serial.println("Water Level: Medium"); }*/
else if (resval>330){ 
    Serial.println("Water Level: High"); 
}
url += String(resval);
Serial.print("\n\n");


int ldr = analogRead(ldrPin);
if (ldr == 0) {
digitalWrite(ledPin, LOW);
Serial.print("Its Night Time, Turn on the LED : ");
} 
else {
digitalWrite(ledPin, HIGH);
Serial.print("Its daytime, Turn off the LED : ");
}
Serial.println(ldr);
url += "+";
url += String(ldr);

Serial.print("\n\n");

soilMoistureValue = analogRead(A0);  //put Sensor insert into soil
soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
Serial.print("Soil Moisture Percentage= ");
Serial.print(soilmoisturepercent);
Serial.print("%\n");
Serial.print("Soil Moisture= ");
Serial.print(soilMoistureValue);
Serial.print("\n\n");
url += "+";
url += String(soilmoisturepercent);

Serial.print("Temperature = ");
Serial.print(temp);
Serial.print("%\n");
Serial.print("humdity= ");
Serial.print(humidity);
Serial.print("\n\n");
url += "+";
url += String(temp);
url += "+";
url += String(humidity);
url +="+"+String(humidity+5); //for npk
Serial.print("All Sensor Data= ");
Serial.println(url);
  // Sending request
send_request(url);
Serial.println("-----------------------------------\n");
    delay(20000);
}
