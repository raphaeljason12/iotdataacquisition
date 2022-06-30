#include <Wire.h>                                                   //Import the required libraries
#include <WiFi.h>
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include "DFRobot_OxygenSensor.h"
#include <WiFiManager.h>



//Test Sensor token: xD1msc4Y-pBcSKW_i88VuDWlpmhokfiJBSUaMfP6XsFs2ffuV-0cJ1GGBaRvWM_ArJjlC7VQljkm7FtSNSKYBw==
#define DEVICE "ESP32"
#define WIFI_SSID "HERJANTOD"                                                                                        //Network Name
#define WIFI_PASSWORD "01234567890"                                                                                //Network Password
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"                                                                                     //InfluxDB v2 server url, e.g. https://eu-central-1-1.aws.cloud2.influxdata.com (Use: InfluxDB UI -> Load Data -> Client Libraries)
#define INFLUXDB_TOKEN "xD1msc4Y-pBcSKW_i88VuDWlpmhokfiJBSUaMfP6XsFs2ffuV-0cJ1GGBaRvWM_ArJjlC7VQljkm7FtSNSKYBw=="                                                                                 //InfluxDB v2 server or cloud API token (Use: InfluxDB UI -> Data -> API Tokens -> <select token>)ROnsR4ico_-eQh9tk5o-h8WDe8IdNeknq0H_ulw12H3ECnDsiSsdSFPRAOvJkzh-osNE3V-tZhrhk23s-Biojg==
#define INFLUXDB_ORG "raphaeljason00@gmail.com"                                                                                     //InfluxDB v2 organization id (Use: InfluxDB UI -> User -> About -> Common Ids )
#define INFLUXDB_BUCKET "TestSensor"                                                                               //InfluxDB v2 bucket name (Use: InfluxDB UI ->  Data -> Buckets)
#define TZ_INFO "AEDT+11"                                                                                                 //InfluxDB v2 timezone


//WifiManager
bool wm_nonblocking = false; // change to true to use non blocking

WiFiManager wm; // global wm instance
WiFiManagerParameter custom_field; // global param ( for non blocking w params )

#define TRIGGER_PIN 15
const int greenled = 18;
const int redled = 19;


//O2 Sensor
#define COLLECT_NUMBER    10             // collect number, the collection range is 1-100.
#define Oxygen_IICAddress ADDRESS_3      //0x73

//BME280
#define SEALEVELPRESSURE_HPA (1013.25)
int temp = 0;                                                       //Variables to store sensor readings
int humid = 0;
int pressure = 0;

//Soil Moisture
const int AirValue = 3570;   //you need to replace this value with Value_1
const int WaterValue = 465;  //you need to replace this value with Value_2
const int moisturepin = 34;
int soilMoistureValue = 0;
int soilmoisturepercent = 0;
int soilmoisturedisplay = 0;

//Rain Sensor
const int rainpin = 35;
int rainamountdisplay = 0;
int rainamountvalue = 0;
int rainamountpercent = 0;

//CO2
const int dioxidepin = 39; //VN
int adcVal = 0;
float voltage = 0;
float voltageDifference = 0;
float concentration = 0;

//Light Senosr BH1750
#include <BH1750.h>

BH1750 lightMeter;
int lightintensity = 0;

//Anemometer
// Pin definitions
# define Hall sensor 23         //  pin 2 ESP32 ( Carefull  to when Writting the ESP32 need to unplug that PIN , because is used internally in the boot)

// Constants definitions
const float pi = 3.14159265;           // Pi Number
int period = 2000;               // Interval of time analized(miliseconds)
int delaytime = 200;             // Time between samples (miliseconds)
int radius = 90;      // Radio from the center to end of CUP in mm
int encoderhole = 18; //hole amount on encoder
// Variable definitions
unsigned int Sample = 0;   // Sample number
unsigned int counter = 0; // magnet counter for sensor
unsigned int RPM = 0;          // Revolutions per minute
int speedwind = 0;         // Wind speed (m/s)
int windspeed = 0;           // Wind speed (km/h)

//UV Sensor SENS43
//int uvpin = 32;

//Volt & Current sensor
int VT_PIN = 32;// connect VT
int AT_PIN = 33; // connect AT

#define ARDUINO_WORK_VOLTAGE 5.0


//SDA 21 -- SCL 22.. rain dry 4096 wet 0

InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);                 //InfluxDB client instance with preconfigured InfluxCloud certificate

Point sensor("weather");                                            //Data point

DFRobot_OxygenSensor Oxygen;
Adafruit_BME280 bme;

void setup()
{
  Serial.begin(115200);                                             //Start serial communication

  //Anemometer
  pinMode(23, INPUT);
  digitalWrite(23, HIGH);

  //Light Sensor BH1750
  Wire.begin();
  lightMeter.begin();

  //O2 Sensor
  while (!Oxygen.begin(Oxygen_IICAddress)) {
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");

  //BME280
  bool status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring or change I2C address!");
    while (1);
  }

  //WifiManager
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP  
  Serial.setDebugOutput(true);  
  delay(3000);
  Serial.println("\n Starting");

  pinMode(TRIGGER_PIN, INPUT);
  pinMode(greenled, OUTPUT);
  pinMode(redled, OUTPUT);
  
  // wm.resetSettings(); // wipe settings

  if(wm_nonblocking) wm.setConfigPortalBlocking(false);

  // add a custom input field
  int customFieldLength = 40;

  const char* custom_radio_str = "<br/><label for='customfieldid'>Custom Field Label</label><input type='radio' name='customfieldid' value='1' checked> One<br><input type='radio' name='customfieldid' value='2'> Two<br><input type='radio' name='customfieldid' value='3'> Three";
  new (&custom_field) WiFiManagerParameter(custom_radio_str); // custom html input
  
  wm.addParameter(&custom_field);
  wm.setSaveParamsCallback(saveParamCallback);

  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);

  // set dark theme
  wm.setClass("invert");

  wm.setConfigPortalTimeout(120); // auto close configportal after n seconds
  
  bool res;
  res = wm.autoConnect("AutoConnectAP","password"); // password protected ap

  if(!res) {
    Serial.println("Failed to connect or hit timeout");
    digitalWrite(redled, HIGH);
    digitalWrite(greenled, LOW);
    // ESP.restart();
  } 
  else {
    //if you get here you have connected to the WiFi    
    Serial.println("connected...yeey :)");
    digitalWrite(redled, LOW);
    digitalWrite(greenled, HIGH);
 
//  

  //ESP32
//  WiFi.mode(WIFI_STA);                                              //Setup wifi connection
//  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
//
//  Serial.print("Connecting to wifi");                               //Connect to WiFi
//  while (wifiMulti.run() != WL_CONNECTED)
//  {
//    Serial.print(".");
//    delay(100);
//  }
//  Serial.println();
//
//  //  sensor.addTag("device", DEVICE);                                   //Add tag(s) - repeat as required
//  //  sensor.addTag("SSID", WIFI_SSID);
//
//  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");                 //Accurate time is necessary for certificate validation and writing in batches
//
//  if (client.validateConnection())                                   //Check server connection
//  {
//    Serial.print("Connected to InfluxDB: ");
//    Serial.println(client.getServerUrl());
//  }
//  else
//  {
//    Serial.print("InfluxDB connection failed: ");
//    Serial.println(client.getLastErrorMessage());
//  }
}

void loop()                                                          //Loop function
{
  //UV Sensor SENS43
//  int uvread = analogRead(uvpin);
//  float uvvalue = uvread * (3.3/4095);
//  int uvindex = uvvalue/10;

  //WifiManager
  if(wm_nonblocking) wm.process(); // avoid delays() in loop when non-blocking and other long running code  
  checkButton();

  //Volt & Current sensor
  int vt_temp = analogRead(VT_PIN);
  int at_temp = analogRead(AT_PIN);

  double voltage = vt_temp * (ARDUINO_WORK_VOLTAGE / 4095.0) * 5;
  double current = at_temp * (ARDUINO_WORK_VOLTAGE / 4095.0);

  int power = voltage * current;


  
  //Anemometer 
  windvelocity();
  RPMcalc();
  WindSpeed();             

  //Light Sensor BH1750
  float lux = lightMeter.readLightLevel();

  //02 Sensor
  float oxygenData = Oxygen.getOxygenData(COLLECT_NUMBER);

  //BME
  temp = bme.readTemperature();                                      //Record temperature
  humid = bme.readHumidity();                                        //Record humidity
  pressure = bme.readPressure() / 100;                               //Record pressure

  //Soil Moisture
  soilMoistureValue = analogRead(moisturepin);
  soilmoisturepercent = map(soilMoistureValue, AirValue, WaterValue, 0, 100);
  if (soilmoisturepercent > 100) {
    soilmoisturedisplay = 100;
  } else if (soilmoisturepercent < 0) {
    soilmoisturedisplay = 0;
  } else if (soilmoisturepercent >= 0 && soilmoisturepercent <= 100) {
    soilmoisturedisplay = soilmoisturepercent;
  }

  //Rain Sensor
  rainamountvalue = analogRead(rainpin);
  rainamountpercent = map(rainamountvalue, 4096, 0, 0, 100);
  if (rainamountpercent > 100) {
    rainamountdisplay = 100;
  } else if (rainamountpercent < 0) {
    rainamountdisplay = 0;
  } else if (rainamountpercent >= 0 && rainamountpercent <= 100) {
    rainamountdisplay = rainamountpercent;
  }

  //CO2 Sensor
//  adcVal = analogRead(dioxidepin);
//  voltage = adcVal * (3.3 / 4095.0);
//
//  if (voltage == 0)
//  {
//    Serial.println("A problem has occurred with the sensor.");
//  }
//  else if (voltage < 0.4)
//  {
//    Serial.println("Pre-heating the sensor...");
//  }
//  else
//  {
//
//    voltageDifference = voltage - 0.4;
//    concentration = (voltageDifference * 5000.0) / 1.6;
//
//
//  }

  sensor.clearFields();                                              //Clear fields for reusing the point. Tags will remain untouched

  sensor.addField("temperature", temp);                              // Store measured value into point
  sensor.addField("humidity", humid);                                // Store measured value into point
  sensor.addField("pressure", pressure);                             // Store measured value into point
  sensor.addField("Soil Moisture", soilmoisturedisplay);
  sensor.addField("rain intensity", rainamountdisplay);
  sensor.addField("oxygen concentration", oxygenData);
//  sensor.addField("CO2 concentration", concentration);
  sensor.addField("light intensity", lux);
  sensor.addField("Wind", windspeed);
//  sensor.addField("uv index", uvindex);
  sensor.addField("Voltage",voltage);
  sensor.addField("Current",current);
  sensor.addField("Watt",power);


  if (wifiMulti.run() != WL_CONNECTED)                               //Check WiFi connection and reconnect if needed
    Serial.println("Wifi connection lost");

  if (!client.writePoint(sensor))                                    //Write data point
  {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  Serial.print("Temp: ");                                            //Display readings on serial monitor
  Serial.println(temp);
  Serial.print("Humidity: ");
  Serial.println(humid);
  Serial.print("Pressure: ");
  Serial.println(pressure);
  Serial.print("Soil Moisture: ");
  Serial.println(soilmoisturedisplay);
  Serial.print("Rain Amount: ");
  Serial.println(rainamountdisplay);
  Serial.print(" Oxygen concentration: ");
  Serial.print(oxygenData);
  Serial.println(" %vol");
//  Serial.print("voltage:");
//  Serial.print(voltage);
//  Serial.println("V");
//  Serial.print("CO2 Concentration:");
//  Serial.print(concentration);
//  Serial.println(" ppm");
  Serial.print("Light Value(lux): ");
  Serial.println(lux);
  Serial.print("Counter: ");
  Serial.print(counter);
  Serial.print(";  RPM: ");
  Serial.print(RPM);
  Serial.print(";  Wind speed: ");
  Serial.print(windspeed);
  Serial.print(" [m/s] ");             
  Serial.println();    
  Serial.print(voltage); 
  Serial.println(" V "); 
  Serial.print(current); 
  Serial.println(" A ");
  Serial.print(power);
  Serial.println(" W");
//  Serial.print("UV index");
//  Serial.println(uvindex);
//  delay(delaytime); 
  
  delay(5000);                                                      //Wait 60 seconds
}

// Measure wind speed
void windvelocity() {
  speedwind = 0;
  windspeed = 0;
  counter = 0;
  attachInterrupt(digitalPinToInterrupt(23), addcount, RISING);
  unsigned long millis();
  long startTime = millis();
  while (millis() < startTime + period) {

  }
}


void RPMcalc() {
  RPM = ((counter/18)*12) / (period / 1000); // Calculate revolutions per minute (RPM)
}

void WindSpeed() {
  windspeed = ((4 * pi * radius * RPM) / 60) / 1000; // Calculate wind speed on m/s

}

void SpeedWind() {
  speedwind = (((4 * pi * radius * RPM) / 60) / 1000) * 3.6; // Calculate wind speed on km/h

}

void addcount() {
  counter++;
}

void checkButton(){
  // check for button press
  if ( digitalRead(TRIGGER_PIN) == LOW ) {
    // poor mans debounce/press-hold, code not ideal for production
    delay(50);
    if( digitalRead(TRIGGER_PIN) == LOW ){
      Serial.println("Button Pressed");
      // still holding button for 3000 ms, reset settings, code not ideaa for production
      delay(3000); // reset delay hold
      if( digitalRead(TRIGGER_PIN) == LOW ){
        Serial.println("Button Held");
        Serial.println("Erasing Config, restarting");
        digitalWrite(redled, HIGH);
        digitalWrite(greenled, HIGH);
        wm.resetSettings();
        ESP.restart();
      }
      
//      // start portal w delay
//      Serial.println("Starting config portal");
//      wm.setConfigPortalTimeout(120);
//      
//      if (!wm.startConfigPortal("OnDemandAP","password")) {
//        Serial.println("failed to connect or hit timeout");
//        delay(3000);
//        // ESP.restart();
//      } else {
//        //if you get here you have connected to the WiFi
//        Serial.println("connected...yeey :)");
//      }
    }
  }
}


String getParam(String name){
  //read parameter from server, for customhmtl input
  String value;
  if(wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void saveParamCallback(){
  Serial.println("[CALLBACK] saveParamCallback fired");
  Serial.println("PARAM customfieldid = " + getParam("customfieldid"));
}
