


#define BLYNK_TEMPLATE_ID "xxxx"
#define BLYNK_TEMPLATE_NAME "Smoker"
#include <blynk.h>
#include "math.h"
#include "LiquidCrystal_I2C_Spark.h"

const int CabFan = D3;
const int CabHeatRelay = D4;
const int SiloRelay = D5;
const int BurnerRelay = D6;
const int AugerRelay = D10;
const int ValueUp = TX;
const int ValueDwn = SCK;
const int NextStepUp = MISO;
const int NextStepDn = RX;
int CabheatLed;
#define ThermistorPIN0 A0 /// cabinet temp
#define ThermistorPIN1 A1
#define ThermistorPIN2 A2
#define ThermistorPIN3 A5
// #define ThermistorPIN4 A4
int servoPin = MOSI;
LiquidCrystal_I2C *lcd;
boolean SmokeEn = false;
boolean Stage1Run = false;
boolean Stage2Run = false;
boolean Stage3Run = false;
boolean Stage4Run = false;
boolean heatenable = false;
boolean StepSequence = false;
boolean Stage1ram = false;
float vin = 3.3;
float resistorB = 100000;
float D = 0.3928220474E-3;  // "A" Coeffecient in Steinhart-Hart Equation BBQchamber probes 100k
float E = 2.612772862E-4;   // "B"
float F = -0.2563516628E-7; // "C"
// float A = 2.4723753e-04; // "A" Coeffecient in Steinhart-Hart Equation ET-73 probes
// float B = 2.3402251e-04; // "B"
// float C = 1.3879768e-07; // "C"
float resistorA = 10000;   // 51000 for james / 10k for jim    50kprobe/jim        // Resistance in ohms of your fixed resistor
float A = 0.8708392063E-3; // "A" Coeffecient in Steinhart-Hart Equation
float B = 2.153670152E-4;  // "B"//////for Inkbird IBT-4XS probe
float C = 1.207550767E-7;  // Jims probes
float f0;
float f1;
float f2;
float f3;
// float f4;
float chksum;
float setramStage1Temp;
float setramStage2Temp;
float setramStage3Temp;
float setramStage4Temp;
float setswStage1Temp;
float setswStage2Temp;
float setswStage3Temp;
float setswStage4Temp;
float setramStage1Counter;
float setramStage2Counter;
float setramStage3Counter;
float setramStage4Counter;
float setswStage1Counter;
float setswStage2Counter;
float setswStage3Counter;
float setswStage4Counter;
float setramalrm1;
float setramalrm2;
float setramalrm3;
float setramalrm4;
float chkalrm1set;
float chkalrm2set;
float chkalrm3set;
float chkalrm4set;
byte ramstatus;
byte stagestatus;
SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);
unsigned long smokepreviousMillis = 0;
unsigned long burnerprevmillis = 0;
unsigned long Stepprevmillis = 0;
unsigned long Stepprevmillis1 = 0;
unsigned long SmokecurrentMillis;
int smokeState = LOW;
bool burnerstate = HIGH;
unsigned long BurnerOfftime = 15000;
unsigned long HighOffTime = 30000;
int countLoop; // This and the next 6 variables are used to shutdown WiFi and restart it if connection to the cloud is lost for over 5 minutes.
int countWIFILoop;
int countLoopWIFI;
int offWIFIcount;
int pos = 0;
int countLoop2;
unsigned long lastCheck;
unsigned long lastWIFICheck;
unsigned long lastCountWiFi;
unsigned long lastCheck1;
unsigned long lastCheck2;
unsigned long Outtimer1;
unsigned long Outtimer2;
unsigned long steptimer1;
unsigned long temptimer2;
unsigned long temptimer3;
unsigned long temptimer4;
unsigned long temptimer5;
unsigned long temptimer6;
unsigned long temptimer7;
unsigned long temptimer8;
unsigned long temptimer9;
unsigned long Displaytimer;
float NextSeq = 11;
unsigned long DlastCheck;
unsigned long DlastCheck1;
unsigned long Dnow1;
unsigned long lastStage1timeCheck;
unsigned long lastStage2timeCheck;
unsigned long lastStage3timeCheck;
unsigned long lastStage4timeCheck;
unsigned long lastBlynkCheck;
float setswsiloOntime;
float setswburnerOntime;
float RamsiloOnTime;
float RamburnerOnTime;
// bool setswStage1a;
bool SwState = LOW;
bool Stepstate = HIGH;
bool swselect;
boolean DisplaySW = true;
Servo myservo;
#define Publishinterval 1800000
unsigned int Publishreset2;
unsigned int Publishreset;
int SmokeLED;
int prev1 = -1;
int buttonpush1 = 0;
int buttonpush2 = 0;
int buttonpush3 = 0;
int buttonpush4 = 0;
int buttonpush5 = 0;
bool writeenable = false;
bool publishen = true;
int writeEnabletimer;
SerialLogHandler logHandler;
void setup()
{
  Serial.begin(9600);
  Log.info("This is info message");
  Log.trace("This is trace message");
  Log.warn("This is warning message");
  Log.error("This is error message, error=%d");
  Log.info("Device OS version: %s", (const char *)System.version());
  WiFi.selectAntenna(ANT_INTERNAL);
  lcd = new LiquidCrystal_I2C(0x27, 20, 4);
  lcd->init();
  lcd->backlight();
  lcd->clear();
  lcd->print("Dons's Smoker 3.0");
 

   Blynk.config("xxxxxxxxxxxxxxxxxxxxxx");//test
  //Blynk.connect(3333);
  Particle.connect();
  pinMode(AugerRelay, OUTPUT);
  pinMode(SiloRelay, OUTPUT);
  pinMode(CabHeatRelay, OUTPUT);
  pinMode(BurnerRelay, OUTPUT);
  pinMode(CabFan, OUTPUT);
  pinMode(ValueUp, INPUT_PULLUP);
  pinMode(ValueDwn, INPUT_PULLUP);
  pinMode(NextStepUp, INPUT_PULLUP);
  pinMode(NextStepDn, INPUT_PULLUP);
  myservo.attach(servoPin);
  digitalWrite(CabHeatRelay, LOW);
  digitalWrite(CabFan, LOW);
  digitalWrite(SiloRelay, LOW);
  digitalWrite(BurnerRelay, LOW);
  EEPROM.get(0, chksum);
  EEPROM.get(4, ramstatus);
  EEPROM.get(8, setramStage1Temp);
  EEPROM.get(12, setramStage2Temp);
  EEPROM.get(16, setramStage1Counter);
  EEPROM.get(20, setramStage2Counter);
  EEPROM.get(24, RamsiloOnTime);
  EEPROM.get(28, RamburnerOnTime);
  EEPROM.get(32, setramStage3Temp);
  EEPROM.get(36, setramStage4Temp);
  EEPROM.get(40, setramStage3Counter);
  EEPROM.get(44, setramStage4Counter);
  EEPROM.get(48, setramalrm1);
  EEPROM.get(52, setramalrm2);
  EEPROM.get(56, setramalrm3);
  EEPROM.get(60, setramalrm4);
  lastBlynkCheck = 0;
  setswStage1Temp = setramStage1Temp;
  setswStage2Temp = setramStage2Temp;
  setswStage3Temp = setramStage3Temp;
  setswStage4Temp = setramStage4Temp;
  setswStage1Counter = setramStage1Counter;
  setswStage2Counter = setramStage2Counter;
  setswStage3Counter = setramStage3Counter;
  setswStage4Counter = setramStage4Counter;
  chkalrm1set = setramalrm1;
  chkalrm2set = setramalrm2;
  chkalrm3set = setramalrm3;
  chkalrm4set = setramalrm4;
  stagestatus = ramstatus;
  setswsiloOntime = RamsiloOnTime;
  setswburnerOntime = RamburnerOnTime;
  // setswStage1a = Stage1ram;
  Publishreset = millis();
  Outtimer2 = millis();
  if (ramstatus == 10)
  {
    Stage1Run = true;
    SmokeEn = true;
  }
}
void loop()
{
   cloudOutDisconnect(); // Function to turn off WiFi if cloud has been disconnected for 5 minutes
  // onWifiAfterOne();     // Function to turn Wifi back on after being off for one minute
  MenuSequence();
  Smoke();
  Burner();
  SmokeOntime();
  HeaterOntime();
  ShowDisplay();
  DisplayIdle();
  ReadSensors();
  Display();
  TemperatureStage1();
  TemperatureStage2();
  TemperatureStage3();
  TemperatureStage4();
  Checkeprom();
  OutsideTempchkStage1();
  OutsideTempchkStage2();
  OutsideTempchkStage3();
  OutsideTempchkStage4();
  Smokeswen();
  TempswSet();
  CounterStage1swset();
  CounterStage2swset();
  CounterStage3swset();
  CounterStage4swset();
  CounterStage1timer();
  CounterStage2timer();
  CounterStage3timer();
  CounterStage4timer();
  Ramstageclear();
  Publish();
  Publish2();
  Blynkchk();
  Smgenwatchdog();
  if (millis() - writeEnabletimer >= 20000)
  {
    writeenable = false;
  }
}
void Publish()
{
  if ((millis() - Publishreset >= Publishinterval) && (Particle.connected))
  {
    // Particle.publish("Chambertemp", String(f0, 1), PRIVATE);
    // Particle.publish("Temp1", String(f2, 1), PRIVATE);////change to f1
    Particle.publish("DEBUG", String(System.freeMemory()), 1, PRIVATE);
    // int rssi = WiFi.RSSI();
    // Particle.publish("RSSI",String(rssi),PRIVATE);
    Publishreset = millis();
  }
}
void Publish2()
{
  if ((f1 >= setramalrm1) && (publishen == true) && (Particle.connected))
  {
    Blynk.logEvent("reached_temperature");
    Particle.publish("SmokerJim", "Temp1_ReachedTemp", PRIVATE);
    publishen = false;
  }
  if ((f2 >= setramalrm2) && (publishen == true) && (Particle.connected))
  {
    Blynk.logEvent("reached_temperature");
    Particle.publish("SmokerJim", "Temp2_ReachedTemp", PRIVATE);
    publishen = false;
  }
  if ((f3 >= setramalrm3) && (publishen == true) && (Particle.connected))
  {
    Blynk.logEvent("reached_temperature");
    Particle.publish("SmokerJim", "Temp3_ReachedTemp", PRIVATE);
    publishen = false;
  }
  if ((millis() - Publishreset2 >= 60000) && (Particle.connected))
  {
    publishen = true;
    Publishreset2 = millis();
  }
}
void Blynkchk()
{
  unsigned long now = millis();
  if (now - lastBlynkCheck >= 10000)
  {
    Blynk.virtualWrite(V1, setramStage1Temp);
    Blynk.virtualWrite(V2, setramStage2Temp);
    Blynk.virtualWrite(V3, setramStage3Temp);
    Blynk.virtualWrite(V4, setramStage4Temp);
    Blynk.virtualWrite(V5, setramStage1Counter);
    Blynk.virtualWrite(V6, setramStage2Counter);
    Blynk.virtualWrite(V7, setramStage3Counter);
    Blynk.virtualWrite(V8, setramStage4Counter);
    Blynk.virtualWrite(V9, f0);
    Blynk.virtualWrite(V10, f1);
    Blynk.virtualWrite(V11, f2);
    Blynk.virtualWrite(V12, f3);
    Blynk.virtualWrite(V14, RamsiloOnTime);
    Blynk.virtualWrite(V15, setramalrm1);
    Blynk.virtualWrite(V16, setramalrm2);
    Blynk.virtualWrite(V17, setramalrm3);
    Blynk.virtualWrite(V18, setramalrm4);
    Blynk.virtualWrite(V19, smokeState);
    Blynk.virtualWrite(V46, countLoop2);
    SmokeLED = SmokeEn;
    Blynk.virtualWrite(V30, SmokeLED);
    CabheatLed = digitalRead(CabHeatRelay);
    Blynk.virtualWrite(V35, CabheatLed);
    if (Stage1Run == true)
    {
      Blynk.virtualWrite(V42, 1);
    }
    if (Stage2Run == true)
    {
      Blynk.virtualWrite(V42, 2);
    }
    if (Stage3Run == true)
    {
      Blynk.virtualWrite(V42, 3);
    }
    if (Stage4Run == true)
    {
      Blynk.virtualWrite(V42, 4);
    }
    if ((Stage1Run == false) && (Stage2Run == false) && (Stage3Run == false) && (Stage4Run == false))
    {
      Blynk.virtualWrite(V42, 0);
    }
    lastBlynkCheck = millis();
  }
}

BLYNK_WRITE(V31)
{
  chkalrm1set = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V32)
{
  chkalrm2set = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V33)
{
  chkalrm3set = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V34)
{
  chkalrm4set = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V40)
{
  if (writeenable == true)
  {
    buttonpush1 = param.asInt();
    Serial.println(buttonpush1);
  }
}
BLYNK_WRITE(V41)
{
  buttonpush2 = param.asInt();
  Serial.println(buttonpush2);
  writeenable = true;
  writeEnabletimer = millis();
}
BLYNK_WRITE(V43)
{
  if (writeenable == true)
  {
    buttonpush3 = param.asInt();
    SmokeEn = true;
  }
}
BLYNK_WRITE(V44)
{
  if (writeenable == true)
  {
    buttonpush4 = param.asInt();
    SmokeEn = false;
  }
}
BLYNK_WRITE(V45)
{
  buttonpush5 = param.asInt();
  if (buttonpush5 == true && writeenable == true)
  {
    setswStage1Counter = -.1;
    setswStage2Counter = -.1;
    setswStage3Counter = -.1;
    setswStage4Counter = -.1;
  }
}
BLYNK_WRITE(V20)
{
  setswStage1Temp = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V21)
{
  setswStage2Temp = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V22)
{
  setswStage3Temp = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V23)
{
  setswStage4Temp = param.asInt(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V24)
{
  setswStage1Counter = param.asDouble(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V25)
{
  setswStage2Counter = param.asDouble(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V26)
{
  setswStage3Counter = param.asDouble(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V27)
{
  setswStage4Counter = param.asDouble(); // assigning incoming value from pin V1 to a variable
}
BLYNK_WRITE(V28)
{
  setswsiloOntime = param.asDouble(); // assigning incoming value from pin V1 to a variable
}
void CounterStage1timer()
{
  if (Stage1Run == true)
  {
    if (setswStage1Counter <= 0)
    {
      Stage2Run = true;
      Stage1Run = false;
      SmokeEn = false;
      Particle.publish("SmokerJim", "Stage 2 started", PRIVATE);
    }
    unsigned long now1 = millis();
    if (now1 - lastStage1timeCheck >= 360000) // 360000
    {
      setswStage1Counter = setswStage1Counter - .1;
      lastStage1timeCheck = millis();
    }
  }
}
void CounterStage2timer()
{
  if (Stage2Run == true)
  {
    if (setswStage2Counter <= 0)
    {
      Stage3Run = true;
      Stage2Run = false;
      Stage1Run = false;
      Particle.publish("SmokerJim", " Stage 3 Started", PRIVATE);
    }
    unsigned long now2 = millis();
    if (now2 - lastStage2timeCheck >= 360000) // 360000
    {
      setswStage2Counter = setswStage2Counter - .1;
      lastStage2timeCheck = millis();
    }
  }
}
void CounterStage3timer()
{
  if (Stage3Run == true)
  {
    if (setswStage3Counter <= 0)
    {
      Stage4Run = true;
      Stage3Run = false;
      Stage2Run = false;
      Stage1Run = false;
      Particle.publish("SmokerJim", " Stage 4 Started", PRIVATE);
    }
    unsigned long now3 = millis();
    if (now3 - lastStage3timeCheck >= 360000) // 360000
    {
      setswStage3Counter = setswStage3Counter - .1;
      lastStage3timeCheck = millis();
    }
  }
}
void CounterStage4timer()
{
  if (Stage4Run == true)
  {
    if (setswStage4Counter <= 0)
    {
      Stage4Run = false;
      Stage3Run = false;
      Stage2Run = false;
      Stage1Run = false;
      digitalWrite(CabHeatRelay, LOW);
      digitalWrite(CabFan, LOW);
      ramstatus = 0;
      EEPROM.put(4, ramstatus);
      Particle.publish("SmokerJim", " Stage 4 ended", PRIVATE);
    }
    unsigned long now4 = millis();
    if (now4 - lastStage4timeCheck >= 360000) // 360000
    {
      setswStage4Counter = setswStage4Counter - .1;
      lastStage4timeCheck = millis();
    }
  }
}
void StepSw()
{
  if (millis() - steptimer1 >= 1000)
  {
    SwState = digitalRead(NextStepUp);
    if (SwState == LOW && Stepstate == HIGH)
    {
      swselect = true;
      Stepstate = LOW;
      steptimer1 = millis();
    }
    else if (SwState == LOW && Stepstate == LOW)
    {
      swselect = false;
      Stepstate = HIGH;
      steptimer1 = millis();
    }
  }
}
void MenuSequence()
{
  if ((digitalRead(NextStepUp) == 0) && (millis() - Outtimer2 >= 800))
  {
    (NextSeq++);
    if (NextSeq > 16)
    {
      NextSeq = 16;
    }
    Serial.println("NextStepUp");
    Serial.println(NextSeq);
    Outtimer2 = millis();
  }
  if ((digitalRead(NextStepDn) == 0) && (millis() - Outtimer2 >= 800))
  {
    (NextSeq--);
    if (NextSeq < 1)
    {
      (NextSeq = 1);
    }
    Serial.println("NextStepDn");
    Outtimer2 = millis();
  }
}

void DisplayIdle()
{
  Dnow1 = millis();
  if (Dnow1 - DlastCheck1 >= 45000)
  {
    (NextSeq = 11);
    DlastCheck1 = millis();
  }
}

void ShowDisplay()
{
  if ((digitalRead(NextStepUp) == 0) || (digitalRead(NextStepDn) == 0) || (digitalRead(ValueDwn) == 0) || (digitalRead(ValueUp) == 0))
  {
    DlastCheck1 = millis();
    Displaytimer = 100;
  }
  else
  {
    Displaytimer = 5000;
  }
}
void HeaterOntime()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 1))
  {
    (setswburnerOntime++); // increase Cool temp
    Outtimer1 = millis();
    Serial.println(setswburnerOntime);
  }
  if (setswburnerOntime > 20)

  {
    setswburnerOntime = 20;
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 1))
  {
    (setswburnerOntime--);
    Outtimer1 = millis();
  }
  if (setswburnerOntime < 10)

  {
    setswburnerOntime = 10;
  }

  EEPROM.get(28, RamburnerOnTime);
  if ((RamburnerOnTime != setswburnerOntime) && (isnan(RamburnerOnTime) == false))
  {
    EEPROM.put(28, setswburnerOntime);
  }
}
void SmokeOntime()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 2))
  {
    (setswsiloOntime++);
    Outtimer1 = millis();
    Serial.println(setswsiloOntime);
  }
  if (setswsiloOntime > 8)

  {
    setswsiloOntime = 8;
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 2))
  {
    (setswsiloOntime--);
    Outtimer1 = millis();
    Serial.println(setswsiloOntime);
  }
  if (setswsiloOntime < 2)

  {
    setswsiloOntime = 2;
  }

  EEPROM.get(24, RamsiloOnTime);
  if ((RamsiloOnTime != setswsiloOntime) && (isnan(RamsiloOnTime) == false))
  {
    EEPROM.put(24, setswsiloOntime);
  }
}
void OutsideTempchkStage1()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 3))
  {
    (setswStage1Temp++);
    Outtimer1 = millis();
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 3))
  {
    (setswStage1Temp--);
    Outtimer1 = millis();
  }
  EEPROM.get(8, setramStage1Temp);
  if ((setramStage1Temp != setswStage1Temp) && (isnan(setramStage1Temp) == false))
  {
    EEPROM.put(8, setswStage1Temp);
  }
}
void OutsideTempchkStage2()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 4))
  {
    (setswStage2Temp++);
    Outtimer1 = millis();
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 4))
  {
    (setswStage2Temp--);
    Outtimer1 = millis();
  }
  EEPROM.get(12, setramStage2Temp);
  if ((setramStage2Temp != setswStage2Temp) && (isnan(setramStage2Temp) == false))
  {
    EEPROM.put(12, setswStage2Temp);
  }
}
void OutsideTempchkStage3()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 5))
  {
    (setswStage3Temp++);
    Outtimer1 = millis();
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 5))
  {
    (setswStage3Temp--);
    Outtimer1 = millis();
  }
  EEPROM.get(32, setramStage3Temp);
  if ((setramStage3Temp != setswStage3Temp) && (isnan(setramStage3Temp) == false))
  {
    EEPROM.put(32, setswStage3Temp);
  }
}
void OutsideTempchkStage4()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 6))
  {
    (setswStage4Temp++);
    Outtimer1 = millis();
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 500) && (NextSeq == 6))
  {
    (setswStage4Temp--);

    Outtimer1 = millis();
  }
  EEPROM.get(36, setramStage4Temp);
  if ((setramStage4Temp != setswStage4Temp) && (isnan(setramStage4Temp) == false))
  {
    EEPROM.put(36, setswStage4Temp);
  }
}
void CounterStage1swset()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 200) && (NextSeq == 7))
  {
    (setswStage1Counter = setswStage1Counter + .1);
    if ((setswStage1Counter < 0) || (setswStage1Counter > 25))
    {
      setswStage1Counter = 1;
    }
    Outtimer1 = millis();
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 200) && (NextSeq == 7))
  {
    (setswStage1Counter = setswStage1Counter - .1);
    if ((setswStage1Counter < 0) || (setswStage1Counter > 25))
    {
      setswStage1Counter = 1;
    }
    Outtimer1 = millis();
  }
  EEPROM.get(16, setramStage1Counter);
  if ((setramStage1Counter != setswStage1Counter) && (isnan(setramStage1Counter) == false))
  {
    EEPROM.put(16, setswStage1Counter);
  }
}
void CounterStage2swset()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 200) && (NextSeq == 8))
  {
    (setswStage2Counter = setswStage2Counter + .1);
    if ((setswStage2Counter < 0) || (setswStage2Counter > 25))
    {
      setswStage2Counter = 1;
    }
    Outtimer1 = millis();
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 200) && (NextSeq == 8))
  {
    (setswStage2Counter = setswStage2Counter - .1);
    if ((setswStage2Counter < 0) || (setswStage2Counter > 25))
    {
      setswStage2Counter = 1;
    }
    Outtimer1 = millis();
  }
  EEPROM.get(20, setramStage2Counter);
  if ((setramStage2Counter != setswStage2Counter) && (isnan(setramStage2Counter) == false))
  {
    EEPROM.put(20, setswStage2Counter);
  }
}
void CounterStage3swset()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 200) && (NextSeq == 9))
  {
    (setswStage3Counter = setswStage3Counter + .1);
    if ((setswStage3Counter < 0) || (setswStage3Counter > 25))
    {
      setswStage3Counter = 1;
    }
    Outtimer1 = millis();
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 200) && (NextSeq == 9))
  {
    (setswStage3Counter = setswStage3Counter - .1);
    if ((setswStage3Counter < 0) || (setswStage3Counter > 25))
    {
      setswStage3Counter = 1;
    }
    Outtimer1 = millis();
  }
  EEPROM.get(40, setramStage3Counter);
  if ((setramStage3Counter != setswStage3Counter) && (isnan(setramStage3Counter) == false))
  {
    EEPROM.put(40, setswStage3Counter);
  }
}
void CounterStage4swset()
{
  if ((digitalRead(ValueUp) == 0) && (millis() - Outtimer1 >= 200) && (NextSeq == 10))
  {
    (setswStage4Counter = setswStage4Counter + .1);
    if ((setswStage4Counter < 0) || (setswStage4Counter > 48))
    {
      setswStage4Counter = 1;
    }
    Outtimer1 = millis();
  }
  if ((digitalRead(ValueDwn) == 0) && (millis() - Outtimer1 >= 200) && (NextSeq == 10))
  {
    (setswStage4Counter = setswStage4Counter - .1);
    if ((setswStage4Counter < 0) || (setswStage4Counter > 25))
    {
      setswStage4Counter = 1;
    }
    Outtimer1 = millis();
  }
  EEPROM.get(44, setramStage4Counter);
  if ((setramStage4Counter != setswStage4Counter) && (isnan(setramStage4Counter) == false))
  {
    EEPROM.put(44, setswStage4Counter);
  }
}
void TempswSet()
{
  unsigned long now = millis();
  if (now - Stepprevmillis1 >= 1000)
  {
    if (((digitalRead(ValueUp) == 0) && (NextSeq == 11)) || ((buttonpush1 == true)))
    {
      (Stage2Run = false);
      (Stage1Run = true);
      (SmokeEn = true);
      ramstatus = 10;
      EEPROM.put(4, ramstatus);
      Particle.publish("SMokerJim", " Stage 1 started", PRIVATE);
      lastStage1timeCheck = millis();
      Stepprevmillis1 = millis();
    }
    if ((digitalRead(ValueDwn) == 0) && (NextSeq == 11))
    {
      (Stage2Run = false);
      (Stage1Run = true);
      (SmokeEn = true);
      ramstatus = 10;
      EEPROM.put(4, ramstatus);
      Particle.publish("SMokerJim", " Stage 1 started", PRIVATE);
      Stepprevmillis1 = millis();
    }
  }
}
void Smokeswen()
{
  if ((digitalRead(ValueUp) == 0) && (NextSeq == 13))
  {
    (SmokeEn = true);
  }
  if ((digitalRead(ValueDwn) == 0) && (NextSeq == 13))
  {
    (SmokeEn = false);
  }
}
void Ramstageclear()
{
  if ((digitalRead(ValueUp) == 0) && (NextSeq == 15))
  {
    stagestatus = 0;
    Serial.println(stagestatus);
    if (stagestatus != ramstatus)
    {
      ramstatus = stagestatus;
    }
    Serial.println(stagestatus);
    EEPROM.put(4, ramstatus);
  }
  if ((digitalRead(ValueDwn) == 0) && (NextSeq == 15))
  {
    stagestatus = 0;
    Serial.println(stagestatus);
    if (stagestatus != ramstatus)
    {
      ramstatus = stagestatus;
    }
    Serial.println(stagestatus);
    EEPROM.put(4, ramstatus);
  }
}
void TemperatureStage1()
{
  if (Stage1Run == true)
  {
    // digitalWrite(CabFan, HIGH);// To allow first stage to run without fan and heat

    if ((f0 < setswStage1Temp) && (millis() - temptimer2 > 5000) && (f0 > 0))
    {

      digitalWrite(CabHeatRelay, HIGH);
      digitalWrite(CabFan, HIGH);
      temptimer3 = millis();
    }
    if ((f0 > setswStage1Temp) && (Stage1Run == true) && (millis() - temptimer3 > 5000))
    {
      digitalWrite(CabHeatRelay, LOW);
      temptimer2 = millis();
    }
  }
}
void TemperatureStage2()
{
  if ((f0 < setswStage2Temp) && (Stage2Run == true) && (millis() - temptimer4 > 5000) && (f0 > 0))
  {
    digitalWrite(CabFan, HIGH);
    digitalWrite(CabHeatRelay, HIGH);
    temptimer5 = millis();
  }
  if ((f0 > setswStage2Temp) && (Stage2Run == true) && (millis() - temptimer5 > 5000))
  {
    digitalWrite(CabHeatRelay, LOW);
    temptimer4 = millis();
  }
}
void TemperatureStage3()
{
  if ((f0 < setswStage3Temp) && (Stage3Run == true) && (millis() - temptimer6 > 5000) && (f0 > 0))
  {
    digitalWrite(CabFan, HIGH);
    digitalWrite(CabHeatRelay, HIGH);
    temptimer7 = millis();
  }
  if ((f0 > setswStage3Temp) && (Stage3Run == true) && (millis() - temptimer7 > 5000))
  {
    digitalWrite(CabHeatRelay, LOW);
    temptimer6 = millis();
  }
}
void TemperatureStage4()
{
  if ((f0 < setswStage4Temp) && (Stage4Run == true) && (millis() - temptimer8 > 5000) && (f0 > 0))
  {
    digitalWrite(CabFan, HIGH);
    digitalWrite(CabHeatRelay, HIGH);
    temptimer9 = millis();
  }
  if ((f0 > setswStage4Temp) && (Stage4Run == true) && (millis() - temptimer9 > 5000))
  {
    digitalWrite(CabHeatRelay, LOW);
    temptimer8 = millis();
  }
}
void Display()
{
  unsigned long Dnow = millis();
  if (Dnow - DlastCheck >= Displaytimer)
  {

    if (NextSeq == 1)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Burner OnTime");
      lcd->setCursor(16, 0);
      lcd->print(setswburnerOntime, 0);
      lcd->setCursor(0, 1);
      lcd->print("Burner  Mem");
      lcd->setCursor(16, 1);
      lcd->print(RamburnerOnTime, 0);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 2)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Set Auger");
      lcd->setCursor(16, 0);
      lcd->print(setswsiloOntime, 0);
      lcd->setCursor(0, 1);
      lcd->print("Auger Memory");
      lcd->setCursor(16, 1);
      lcd->print(RamsiloOnTime, 0);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 3)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Set Stage1 Temp");
      lcd->setCursor(16, 0);
      lcd->print(setswStage1Temp, 0);
      lcd->setCursor(0, 1);
      lcd->print("Stage1 Memory");
      lcd->setCursor(16, 1);
      lcd->print(setramStage1Temp, 0);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 4)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Set Stage2 Temp");
      lcd->setCursor(16, 0);
      lcd->print(setswStage2Temp, 0);
      lcd->setCursor(0, 1);
      lcd->print("Stage2 Temp Mem");
      lcd->setCursor(16, 1);
      lcd->print(setramStage2Temp, 0);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 5)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Set Stage3 Temp");
      lcd->setCursor(16, 0);
      lcd->print(setswStage3Temp, 0);
      lcd->setCursor(0, 1);
      lcd->print("Stage3 Temp Mem");
      lcd->setCursor(16, 1);
      lcd->print(setramStage3Temp, 0);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 6)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Set Stage4 Temp");
      lcd->setCursor(16, 0);
      lcd->print(setswStage4Temp, 0);
      lcd->setCursor(0, 1);
      lcd->print("Stage4 Temp Mem");
      lcd->setCursor(16, 1);
      lcd->print(setramStage4Temp, 0);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 7)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("CounterStage1");
      lcd->setCursor(16, 0);
      lcd->print(setswStage1Counter, 1);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 8)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("CounterStage2");
      lcd->setCursor(16, 0);
      lcd->print(setswStage2Counter, 1);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 9)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("CounterStage3");
      lcd->setCursor(16, 0);
      lcd->print(setswStage3Counter, 1);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 10)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("CounterStage4");
      lcd->setCursor(16, 0);
      lcd->print(setswStage4Counter, 1);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 11)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Stage");
      lcd->setCursor(0, 2);
      int rssi = WiFi.RSSI();
      lcd->print(rssi);
      lcd->setCursor(12, 2);
      lcd->print("Push Up");
      lcd->setCursor(12, 3);
      lcd->print("To Start");
      lcd->setCursor(6, 0);

      if (Stage1Run == true)
      {
        lcd->print("One");
      }
      if (Stage2Run == true)
      {
        lcd->print("Two");
      }
      if (Stage3Run == true)
      {
        lcd->print("Three");
      }
      if (Stage4Run == true)
      {
        lcd->print("Four");
      }
      lcd->setCursor(12, 0);
      lcd->print("CT");
      if (f0 < 0)
      {
        lcd->setCursor(15, 0);
        lcd->print("NC");
      }
      else
      {
        lcd->setCursor(15, 0);
        lcd->print(f0, 1);
      }
      lcd->setCursor(0, 1);
      lcd->print("TP1");
      if (f1 < 0)
      {
        lcd->setCursor(5, 1);
        lcd->print("NC");
      }
      else
      {
        lcd->setCursor(5, 1);
        lcd->print(f1, 0);
      }
      lcd->setCursor(9, 1);
      lcd->print("TP2");
      if (f2 < 0)
      {
        lcd->setCursor(13, 1);
        lcd->print("NC");
      }
      else
      {
        lcd->setCursor(13, 1);
        lcd->print(f2, 0);
      }
      if (Stage1Run == true)
      {
        lcd->setCursor(0, 2);
        lcd->print("Counter 1");
        lcd->setCursor(0, 3);
        lcd->print(setswStage1Counter, 1);
      }
      if (Stage2Run == true)
      {
        lcd->setCursor(0, 2);
        lcd->print("Counter 2");
        lcd->setCursor(0, 3);
        lcd->print(setswStage2Counter, 1);
      }
      if (Stage3Run == true)
      {
        lcd->setCursor(0, 2);
        lcd->print("Counter 3");
        lcd->setCursor(0, 3);
        lcd->print(setswStage3Counter, 1);
      }
      if (Stage4Run == true)
      {
        lcd->setCursor(0, 2);
        lcd->print("Counter 4");
        lcd->setCursor(0, 3);
        lcd->print(setswStage4Counter, 1);
      }
    }
    if (NextSeq == 12)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Cabinet Temp");
      lcd->setCursor(14, 0);
      lcd->print(f0, 1);
      lcd->setCursor(0, 1);
      lcd->print("Temp1 Probe");
      lcd->setCursor(14, 1);
      lcd->print(f1, 1);
      lcd->setCursor(0, 2);
      lcd->print("Temp 2 Probe");
      lcd->setCursor(14, 2);
      lcd->print(f2, 1);
      lcd->setCursor(0, 3);
      lcd->print("Tp3");
      lcd->setCursor(4, 3);
      lcd->print(f3, 1);
      lcd->setCursor(10, 3);
    }
    if (NextSeq == 13)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Smoke Enable");
      lcd->setCursor(16, 0);
      lcd->print(SmokeEn);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    if (NextSeq == 14)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("CountersReset");
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
      lcd->setCursor(0, 3);
      lcd->print("Push Up to reset Count");
      if ((digitalRead(ValueUp) == 0) && (NextSeq == 14))
      {
        setswStage1Counter = -.1;
        setswStage2Counter = -.1;
        setswStage3Counter = -.1;
        setswStage4Counter = -.1;
        lcd->setCursor(0, 2);
        lcd->print("Ct1");
        lcd->setCursor(6, 2);
        lcd->print(setswStage1Counter, 1);
        lcd->setCursor(11, 2);
        lcd->print("Ct2");
        lcd->setCursor(16, 2);
        lcd->print(setswStage2Counter, 1);
        lcd->setCursor(0, 3);
        lcd->print("Ct3");
        lcd->setCursor(6, 3);
        lcd->print(setswStage3Counter, 1);
        lcd->setCursor(11, 3);
        lcd->print("Ct4");
        lcd->setCursor(16, 3);
        lcd->print(setswStage4Counter, 1);
      }
    }
    if (NextSeq == 15)
    {
      lcd->clear();
      lcd->setCursor(0, 0);
      lcd->print("Stagestatus");
      lcd->setCursor(16, 0);
      lcd->print(stagestatus);
      lcd->setCursor(0, 1);
      lcd->print("Ramstatus");
      lcd->setCursor(16, 1);
      lcd->print(ramstatus);
      lcd->setCursor(0, 2);
      lcd->print("Sequence Step");
      lcd->setCursor(16, 2);
      lcd->print(NextSeq, 0);
    }
    DlastCheck = millis();
  }
}
void Burner()
{

  unsigned long currentMillis = millis();

  if ((burnerstate == HIGH) && (currentMillis - burnerprevmillis >= (setswburnerOntime * 1000) && (SmokeEn == TRUE)))
  {
    burnerstate = LOW;
    burnerprevmillis = currentMillis;
    digitalWrite(BurnerRelay, burnerstate);
  }
  else if ((burnerstate == LOW) && (currentMillis - burnerprevmillis >= BurnerOfftime) && (SmokeEn == TRUE))
  {
    burnerstate = HIGH;
    burnerprevmillis = currentMillis;
    digitalWrite(BurnerRelay, burnerstate);
  }
  if (SmokeEn == false)
  {
    digitalWrite(BurnerRelay, LOW);
  }
}
void Smgenwatchdog()
{
  unsigned long now = millis(); // make sure smokegen resets if auger runs too long
  if ((now - lastCheck2 >= 1000) && (digitalRead(AugerRelay) == 1))
  {
    countLoop2++;
    lastCheck2 = millis();
    if (countLoop2 >= 4)
    {
      smokepreviousMillis = SmokecurrentMillis;
      digitalWrite(AugerRelay, LOW);
      digitalWrite(SiloRelay, LOW);
      countLoop2 = 0;
      smokeState = LOW;
      lastCheck2 = millis();
    }
  }
}
void Smoke()
{
  SmokecurrentMillis = millis();

  if ((smokeState == LOW) && (SmokecurrentMillis - smokepreviousMillis >= HighOffTime) && (SmokeEn == TRUE))
  {
    smokeState = HIGH;
    myservo.attach(servoPin);
    for (pos = 40; pos >= 0; pos -= 1)
    {
      myservo.write(pos);
      delay(15);
    }
    for (pos = 0; pos <= 40; pos += 1)
    {
      myservo.write(pos);
      delay(15);
    }
    smokepreviousMillis = SmokecurrentMillis;
    digitalWrite(AugerRelay, HIGH);
    digitalWrite(SiloRelay, HIGH);
    // myservo.detach();
  }
  else if ((smokeState == HIGH) && (SmokecurrentMillis - smokepreviousMillis >= (setswsiloOntime * 1000)))
  {
    smokeState = LOW;
    smokepreviousMillis = smokepreviousMillis;
    digitalWrite(AugerRelay, LOW);
    digitalWrite(SiloRelay, LOW);
    countLoop2 = 0;
    lastCheck2 = millis();
  }
}
void ReadSensors()
{
  float a0 = analogRead(ThermistorPIN0);
  float v0 = a0 * .000805;
  v0 = vin - v0;
  float r0 = (((resistorB * vin) / v0) - resistorB);
  float logr0 = log(r0);
  float logcubed0 = logr0 * logr0 * logr0;
  float k0 = 1.0 / (D + (E * logr0) + (F * logcubed0));
  f0 = (1.8 * (k0 - 273)) + 32;
  float a1 = analogRead(ThermistorPIN1);                // This reads the "voltage" value on A0. Value is actually divided into 1024 steps from 0-1023.
  float v1 = a1 * .000805;                              // Converts A0 value to an actual voltage (3.3V / 4096 steps = .000805V/step.
  v1 = vin - v1;                                        // Subtract 3.3 from calculated to get real voltage
  float r1 = (((resistorA * vin) / v1) - resistorA);    // This reads the "voltage" value on A0. Value is actually divided into 4096 steps.
  float logr1 = log(r1);                                // Calculates resistance value of thermistor based on fixed resistor value and measured voltage
                                                        // Natural log of thermistor resistance used in Steinhart-Hart Equation
  float logcubed1 = logr1 * logr1 * logr1;              // The cube of the above value
  float k1 = 1.0 / (A + (B * logr1) + (C * logcubed1)); // Steinhart-Hart Equation to calculate temperature in Kelvin
  f1 = (1.8 * (k1 - 273)) + 32;
  float a2 = analogRead(ThermistorPIN2);
  float v2 = a2 * .000805;
  v2 = vin - v2;
  float r2 = (((resistorA * vin) / v2) - resistorA);
  float logr2 = log(r2);
  float logcubed2 = logr2 * logr2 * logr2;
  float k2 = 1.0 / (A + (B * logr2) + (C * logcubed2));
  f2 = (1.8 * (k2 - 273)) + 32;
  float a3 = analogRead(ThermistorPIN3);
  float v3 = a3 * .000805;
  v3 = vin - v3;
  float r3 = (((resistorA * vin) / v3) - resistorA);
  float logr3 = log(r3);
  float logcubed3 = logr3 * logr3 * logr3;
  float k3 = 1.0 / (A + (B * logr3) + (C * logcubed3));
  f3 = (1.8 * (k3 - 273)) + 32;
}

void cloudOutDisconnect()
{
  if ((!Particle.connected()) && (WiFi.ready()))
  {
    unsigned long now = millis();
    if (now - lastCheck >= 1000)
    {
      countLoop++;
      Blynk.disconnect();
      // Serial.print("countLoop");
      // Serial.println(countLoop);
      lastCheck1 = millis();
      lastCheck = millis();
      if (countLoop >= 60)
      {
       // WiFi.off();
        // Serial.println("WiFi turned off at ");
        //Serial.println(Time.now());
        offWIFIcount++;
        countLoop = 0;
      }
    }
  }
  if ((!Blynk.connected()) && (Particle.connected()))
  {
    unsigned long now1 = millis();
    if (now1 - lastCheck1 >= 10000)
    {
      Blynk.connect(3333);
      Blynk.run();
      //Serial.println("blynk connect/run");
      lastCheck1 = millis();
    }
  }
  if (Particle.connected() && Blynk.connected())
  {
    // Serial.print("Pconnected");
    Blynk.run();
  }
}

void onWifiAfterOne()
{
  if (!WiFi.ready())
  {
    unsigned long now = millis();
    if (now - lastWIFICheck >= 1000)
    {
      countWIFILoop++;
      Serial.print("Countloop+ ");
      lastWIFICheck = millis();
      if (countWIFILoop >= 10) // 60
      {
        WiFi.on();
        Particle.connect();
        Serial.print("WiFi turned back on at ");
        Serial.println(Time.now());
        countWIFILoop = 0;
      }
    }
  }
  else
    (countWIFILoop = 0);
}
void Checkeprom()
{
  float a = 150;
  float b = 80;
  float c = 2;
  float d = 3;
  float e = 15;
  if (chksum != 10)
  {
    setramStage1Temp = 80;
    setramStage2Temp = 100;
    chksum = 10;
    EEPROM.put(0, chksum);
  }
  if (isnan(setramStage1Temp))
  {
    EEPROM.put(8, a);
  }
  if (isnan(setswStage1Temp))
  {
    setswStage1Temp = a;
  }
  if (isnan(setramStage2Temp))
  {
    EEPROM.put(12, b);
  }
  if (isnan(setswStage2Temp))
  {
    setswStage2Temp = b;
  }
  if (isnan(setramStage3Temp))
  {
    EEPROM.put(32, b);
  }
  if (isnan(setswStage3Temp))
  {
    setswStage3Temp = b;
  }
  if (isnan(setramStage4Temp))
  {
    EEPROM.put(36, b);
  }
  if (isnan(setswStage4Temp))
  {
    setswStage4Temp = b;
  }
  if (isnan(setramStage1Counter))
  {
    EEPROM.put(16, c);
  }
  if (isnan(setramStage2Counter))
  {
    EEPROM.put(20, c);
  }
  if (isnan(setramStage3Counter))
  {
    EEPROM.put(40, c);
  }
  if (isnan(setramStage4Counter))
  {
    EEPROM.put(44, c);
  }
  if (isnan(RamsiloOnTime))
  {
    EEPROM.put(24, d);
  }
  if (isnan(setswsiloOntime))
  {
    setswsiloOntime = d;
  }
  if (isnan(RamburnerOnTime))
  {
    EEPROM.put(28, e);
  }
  if (isnan(setswburnerOntime))
  {
    setswburnerOntime = e;
  }
  if (isnan(setramalrm1))
  {
    EEPROM.put(48, a);
  }
  if (isnan(setramalrm2))
  {
    EEPROM.put(52, a);
  }
  if (isnan(setramalrm3))
  {
    EEPROM.put(56, a);
  }
  if (isnan(setramalrm4))
  {
    EEPROM.put(60, a);
  }
  EEPROM.get(48, setramalrm1);
  if ((setramalrm1 != chkalrm1set) && (isnan(setramalrm1) == false))
  {
    EEPROM.put(48, chkalrm1set);
  }
  EEPROM.get(52, setramalrm2);
  if ((setramalrm2 != chkalrm2set) && (isnan(setramalrm2) == false))
  {
    EEPROM.put(52, chkalrm2set);
  }
  EEPROM.get(56, setramalrm3);
  if ((setramalrm3 != chkalrm3set) && (isnan(setramalrm3) == false))
  {
    EEPROM.put(56, chkalrm3set);
  }
  EEPROM.get(60, setramalrm4);
  if ((setramalrm4 != chkalrm4set) && (isnan(setramalrm4) == false))
  {
    EEPROM.put(60, chkalrm4set);
  }
}