#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <MAX30100_PulseOximeter.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MPU6050.h>
#include <TinyGPS++.h>

// ================= DISPLAY =================
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ================= MAX30100 =================
PulseOximeter pox;

// ================= BATTERY ADC =================
#define BATTERY_PIN 34

// divider values
const float R1 = 100000.0;
const float R2 = 47000.0;

float batteryVoltage = 0;
int batteryPercent = 0;



// ================= DS18B20 =================
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// ================= MPU6050 =================
MPU6050 mpu;
float totalAccel = 1.0;

// ================= GSM =================
HardwareSerial sim800(2); // SIM800L
#define GSM_RX 16
#define GSM_TX 17

// ================= GPS =================
HardwareSerial gpsSerial(0);
TinyGPSPlus gps;
#define GPS_RX 3
#define GPS_TX 1

double latitude = 0;
double longitude = 0;
bool gpsFix = false;

// ================= BUTTON & Buzzer =================
#define ALERT_BUTTON 27
#define BUZZER 14

// ================= SENSOR DATA =================
float filteredHR = 0;
float filteredSpO2 = 0;
float bodyTemp = 0.0;

bool fingerDetected = false;
bool fallDetected = false;
bool sendingSMS = false;

// ================= TIMERS =================
unsigned long lastHRRead = 0;
unsigned long lastTempTrigger = 0;
unsigned long lastTempRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastMPURead = 0;
unsigned long bootStart = 0;

int gsmLevel = 0;

enum State { BOOT, RUN };
State currentState = BOOT;

// =====================================================
void onBeatDetected()
{
  Serial.println("Beat!");
}


// =====================================================
// GSM SEND COMMAND
// =====================================================
void gsmCommand(String cmd, int delayTime)
{
  sim800.println(cmd);
  delay(delayTime);
  while(sim800.available())
  {
    Serial.write(sim800.read());
  }
}



// =====================================================
// SEND ALERT SMS
// =====================================================
void sendAlertSMS()
{
  sendingSMS = true;
  
  String message = "ALERT! FALL DETECTED\n";
  message += "HR: " + String(filteredHR,0) + " bpm\n";
  message += "SpO2: " + String(filteredSpO2,0) + "%\n";
  message += "Temp: " + String(bodyTemp,1) + " C\n";
  message += "Accel: " + String(totalAccel,2) + "G\n";

  if(gpsFix)
  {
    message += "Location:\n";
    message += "https://maps.google.com/?q=";
    message += String(latitude,6) + "," + String(longitude,6);
  }
  else
  {
    message += "GPS: Not Fixed";
  }
  
  gsmCommand("AT",1000);
  gsmCommand("AT+CMGF=1",1000);

  sim800.println("AT+CMGS=\"+917904311085\""); // CHANGE NUMBER
  delay(1000);

  sim800.print(message);
  delay(500);
  sim800.write(26);
  delay(6000);

  sendingSMS = false;
}


// =====================================================
// GPS READ
// =====================================================
void GPS_Location_Read()
{
  while (gpsSerial.available())
  {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated())
  {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  }

  gpsFix = gps.location.isValid();
}


// =====================================================
// DRAW GSM ICON
// =====================================================
void drawGSM(int level)
{
  int x = 2;
  int y = 10;

  for(int i=0;i<4;i++)
  {
    int height = (i+1)*3;
    if(i < level)
      u8g2.drawBox(x + i*5, y-height, 3, height);
    else
      u8g2.drawFrame(x + i*5, y-height, 3, height);
  }
}

// ================= BATTERY ICON =================
void drawBattery(int percent)
{
  int x = 100;
  int y = 2;

  u8g2.drawFrame(x,y,20,8);
  u8g2.drawBox(x+20,y+2,2,4);

  int fill = map(percent,0,100,0,18);
  u8g2.drawBox(x+1,y+1,fill,6);
}


// =====================================================
// GPS STATUS INDICATOR
// =====================================================
void drawGPSStatus()
{
  if(gpsFix)
    u8g2.drawStr(70,8,"GPS");
  else
    u8g2.drawStr(65,8,"NO GPS");
}

// ================= READ VOLTAGE =================
float readBatteryVoltage()
{
  uint32_t sum = 0;

  for(int i=0;i<20;i++)
  {
    sum += analogRead(BATTERY_PIN);
    delay(2);
  }

  float raw = sum / 20.0;

  float adcVoltage = raw * 3.3 / 4095.0;

  float battery = adcVoltage * ((R1 + R2) / R2);

  return battery;
}


// ================= VOLTAGE → PERCENT =================
int voltageToPercent(float v)
{
  float percent = (v - 3.0) * 100.0 / (4.2 - 3.0);

  percent = constrain(percent,0,100);

  return (int)percent;
}



// =====================================================
// FALL DETECTION
// =====================================================
void checkFall()
{
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  totalAccel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);

  // Print all values
  Serial.print("AX: ");
  Serial.print(accelX, 2);
  Serial.print("  AY: ");
  Serial.print(accelY, 2);
  Serial.print("  AZ: ");
  Serial.print(accelZ, 2);
  Serial.print("  | Total G: ");
  Serial.println(totalAccel, 2);

  if(totalAccel > 1.5 || totalAccel < 0.4)
  {
    fallDetected = true;
    Serial.println("⚠ FALL DETECTED!");
  }
  else
  {
    fallDetected = false;
  }
}

// =====================================================
void drawBootScreen()
{
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_chroma48medium8_8r);
    u8g2.drawStr(15,25," Smart Silver");
    u8g2.drawStr(10,40,"Guardian System");
  } while (u8g2.nextPage());
}

// =====================================================
void drawSendingScreen()
{
  u8g2.firstPage();
  do
  {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(20,30,"Sending Alert...");
    u8g2.drawStr(25,50,"Please Wait");
  } while (u8g2.nextPage());
}

// =====================================================
void drawMainUI()
{
  u8g2.firstPage();
  do
  {
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_5x7_tr);
    
    drawGSM(gsmLevel);
    drawBattery(batteryPercent);
    drawGPSStatus();
    
    u8g2.drawStr(35,8,"Band");
    u8g2.drawLine(0,12,127,12);

    // Temperature
    char tempStr[20];
    sprintf(tempStr,"Temp: %.1f C",bodyTemp);
    u8g2.drawStr(2,25,tempStr);

    // Heart + SpO2
    char hrStr[20];
    char spo2Str[20];

    if(!fingerDetected)
    {
      sprintf(hrStr,"HR: --");
      sprintf(spo2Str,"SpO2: --");
    }
    else
    {
      sprintf(hrStr,"HR: %.0f bpm",filteredHR);
      sprintf(spo2Str,"SpO2: %.0f %%",filteredSpO2);
    }

    u8g2.drawStr(2,40,hrStr);
    u8g2.drawStr(2,55,spo2Str);

    // Acceleration display (Top Right)
    char accelStr[20];
    sprintf(accelStr,"G: %.2f", totalAccel);
    u8g2.drawStr(80,40,accelStr);

    // Fall Status
    if(fallDetected)
      u8g2.drawStr(80,55,"FALL!");
    else
      u8g2.drawStr(80,55,"SAFE");

  } while (u8g2.nextPage());
}

// =====================================================
void setup()
{
  Serial.begin(115200);

  pinMode(ALERT_BUTTON, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
  
    // GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  // GSM
  sim800.begin(9600, SERIAL_8N1, GSM_RX, GSM_TX);
  
  Wire.begin(21,22);
  Wire.setClock(50000);   // lower speed
  
  u8g2.begin();
  
  tempSensor.begin();
  tempSensor.setWaitForConversion(false);
  
  // ---- MPU FIRST ----
  mpu.initialize();
  
  if(!mpu.testConnection())
  {
    Serial.println("MPU6050 FAILED");
  }
  else
  {
    Serial.println("MPU6050 OK");
  }
  
  // ---- MAX30100 AFTER ----
  if(!pox.begin())
  {
    Serial.println("MAX30100 FAILED");
    while(1);
  }
  
  Serial.println("MAX30100 OK");
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  bootStart = millis();
}

// =====================================================
void loop()
{
  unsigned long now = millis();
  digitalWrite(BUZZER, LOW); 

  batteryVoltage = readBatteryVoltage();

  batteryPercent = voltageToPercent(batteryVoltage);
  
  // MUST RUN ALWAYS
  pox.update();
  GPS_Location_Read();

  
   
  // BOOT SCREEN
  if(currentState == BOOT)
  {
    drawBootScreen();
    if(now - bootStart > 2000)
      currentState = RUN;
    return;
  }

  // HEART RATE
  if(now - lastHRRead > 100)
  {
    float newHR = pox.getHeartRate();
    float newSpO2 = pox.getSpO2();

    if(newHR > 40 && newHR < 180)
      filteredHR = (filteredHR * 0.7) + (newHR * 0.3);

    if(newSpO2 > 80 && newSpO2 <= 100)
      filteredSpO2 = (filteredSpO2 * 0.7) + (newSpO2 * 0.3);

    fingerDetected = (filteredHR > 40);

    Serial.print("HR: ");
    Serial.print(filteredHR);
    Serial.print("  SpO2: ");
    Serial.println(filteredSpO2);

    lastHRRead = now;
  }

  // TEMPERATURE
  if(now - lastTempTrigger > 1000)
  {
    tempSensor.requestTemperatures();
    lastTempTrigger = now;
  }

  if(now - lastTempRead > 1200)
  {
    bodyTemp = tempSensor.getTempCByIndex(0);
    lastTempRead = now;
  }

  // MPU6050
  if(now - lastMPURead > 200)
  {
    checkFall();
    lastMPURead = now;
  }

   // BUTTON + FALL ALERT
  if(fallDetected || digitalRead(ALERT_BUTTON) == LOW && !sendingSMS)
  {
    digitalWrite(BUZZER, HIGH); 
    drawSendingScreen();
    sendAlertSMS();
  }

  if(!sendingSMS && now - lastDisplayUpdate > 800)
  {
    drawMainUI();
    lastDisplayUpdate = now;
  }
}
