#include <WiFi.h>
#include <HTTPClient.h>
#include "Adafruit_VL53L0X.h"
#include <QTRSensors.h>

///////////////  ------ SET THE ID OF THE ROBOT
int Id = 2;
///////////////


#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31
#define SHT_SENSOR1 13
#define SHT_SENSOR2 23

#define LED_PIN1 5
#define LED_PIN2 18
#define LED_PIN3 19

#define MOTOR1_PIN1 0
#define MOTOR1_PIN2 2
#define MOTOR1_PWM 4
#define MOTOR2_PIN1 16
#define MOTOR2_PIN2 17
#define MOTOR2_PWM 15

#define NUM_SENSORS   8      // number of sensors used
#define TIMEOUT       2500   // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   12     // emitter is controlled by digital pin 12
#define IR_SENSOR1 34
#define IR_SENSOR2 35
#define IR_SENSOR3 32
#define IR_SENSOR4 33
#define IR_SENSOR5 25
#define IR_SENSOR6 26
#define IR_SENSOR7 27
#define IR_SENSOR8 14

#define BUTTON_PIN 7  //D0

// Setting PWM properties
const int freq = 20000;
const int PWMChannel_1 = 0;
const int PWMChannel_2 = 1;
const int resolution = 8;

QTRSensorsRC qtrrc((unsigned char[]) {
  IR_SENSOR1, IR_SENSOR2, IR_SENSOR3, IR_SENSOR4, IR_SENSOR5, IR_SENSOR6, IR_SENSOR7, IR_SENSOR8
}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

//const char* SsId = "Crybe";
//const char* Password =  "Crybe123456";

//const char* SsId = "Internet";
//const char* Password =  "";

const char* SsId = "Kucko4";
const char* Password =  "kkuucckkoo";


Adafruit_VL53L0X DistanceSensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X DistanceSensor2 = Adafruit_VL53L0X();

unsigned int SensorValues[8] = {0};
int distance1;
int distance2;
int positionX;
int positionY;
int leftMotorPwm;
int rightMotorPwm;
int errorrr;



//---------------------------------------------------------------------------------------------------Setup---------------------------------------------
void setup() {

  Serial.begin(115200);
  delay(4000);
  
  while (! Serial) { delay(1); }
  
// sets the pins as in/outputs:
  pinMode(SHT_SENSOR1, OUTPUT);
  pinMode(SHT_SENSOR2, OUTPUT);
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(MOTOR1_PIN1, OUTPUT);
  pinMode(MOTOR1_PIN2, OUTPUT);
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR2_PIN1, OUTPUT);
  pinMode(MOTOR2_PIN2, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

// set all output to LOW
  digitalWrite(SHT_SENSOR1, LOW);
  digitalWrite(SHT_SENSOR2, LOW);
  digitalWrite(LED_PIN1, LOW);
  digitalWrite(LED_PIN2, LOW);
  digitalWrite(LED_PIN3, LOW);
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  digitalWrite(MOTOR1_PWM, LOW);
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
  digitalWrite(MOTOR1_PWM, LOW);

  digitalWrite(EMITTER_PIN, HIGH);
  
// configure LED PWM functionalitites
  ledcSetup(PWMChannel_1, freq, resolution);
  ledcSetup(PWMChannel_2, freq, resolution);

// attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR1_PWM, PWMChannel_1);
  ledcAttachPin(MOTOR2_PWM, PWMChannel_2);

  Set_distance_sensors();
  Set_wifi(SsId, Password);
}
//----------------------------------------------------------------------------------------------------Loop--------------------------------------------
void loop() {

  distance1 = Get_distance1();
  distance2 = Get_distance2();
  leftMotorPwm = random(300);
  rightMotorPwm = random(300);
  errorrr = random(300);




  send_data(Id, distance1, distance2, SensorValues, leftMotorPwm, rightMotorPwm, errorrr);
  delay(100);
}
//----------------------------------------------------------------------------------------------------Set Wifi--------------------------------------------
void Set_wifi(const char* SsId, const char* Password) {
  WiFi.begin(SsId, Password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}
//-------------------------------------------------------------------------------------------------------Set distance sensors----------------------------------------
void Set_distance_sensors() {

  // all reset
  digitalWrite(SHT_SENSOR1, LOW);
  digitalWrite(SHT_SENSOR2, LOW);
  delay(10);

  // all unreset
  digitalWrite(SHT_SENSOR1, HIGH);
  digitalWrite(SHT_SENSOR2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_SENSOR1, HIGH);
  digitalWrite(SHT_SENSOR2, LOW);
  delay(10);

  // initing LOX1
  if (!DistanceSensor1.begin(SENSOR1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    //while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_SENSOR2, HIGH);
  delay(10);

  //initing LOX2
  if (!DistanceSensor2.begin(SENSOR2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    //while(1);
  }
}

//-------------------------------------------------------------------------------------------------------------measure_distance 1-----------------------------------
int Get_distance1() {

  VL53L0X_RangingMeasurementData_t MeasureDistance1;
  Serial.print("Reading a measurement... ");
  DistanceSensor1.rangingTest(&MeasureDistance1, false); // pass in 'true' to get debug data printout!

  if (MeasureDistance1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("distance1 (mm): ");
    Serial.println(MeasureDistance1.RangeMilliMeter);
  } else {
    Serial.println("distance 1 out of range ");
  }

  //return MeasureDistance1.RangeMilliMeter;
  return random(200);
}
//--------------------------------------------------------------------------------------------------------------measure_distance 2----------------------------------
int Get_distance2() {

  VL53L0X_RangingMeasurementData_t MeasureDistance2;
  Serial.print("Reading a measurement... ");
  DistanceSensor2.rangingTest(&MeasureDistance2, false); // pass in 'true' to get debug data printout!

  if (MeasureDistance2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("distance2 (mm): ");
    Serial.println(MeasureDistance2.RangeMilliMeter);
  } else {
    Serial.println("distance 2 out of range ");
  }

  //return MeasureDistance2.RangeMilliMeter;
  return random(200);
}

void Set_QTR() {
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}


//--------------------------------------------------------------------------------------------------------------------read_position_x----------------------------
int Get_position_x() {
  int value = random(100);
  return value;
}
//--------------------------------------------------------------------------------------------------------------------read_position_y----------------------------
int Get_position_y() {
  int value = random(100);
  return value;
}
//--------------------------------------------------------------------------------------------------------------------read_ir----------------------------
int Get_ir_sensor() {

  unsigned int position = qtrrc.readLine(sensorValues);
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}
//-------------------------------------------------------------------------------------------------------------------- Button ----------------------------
bool Get_Button()
{
  if (digitalRead(BUTTON_PIN))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
//-------------------------------------------------------------------------------------------------------------------- Set left PWM ----------------------------
void Set_left_motor(int direcrion,int pwm)
{
  switch (direcrion)
  {
  case 0:
    // megall
  digitalWrite(MOTOR1_PIN1, LOW);
  digitalWrite(MOTOR1_PIN2, LOW);
  ledcWrite(PWMChannel_1, pwm);
  Serial.println("Megall");
    break;
  case 1:
    // elorre
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, LOW);  
    ledcWrite(PWMChannel_1, pwm);
  Serial.println("Elorre");
    break;
  case 2:
    // hatra
    digitalWrite(MOTOR1_PIN1, LOW);
    digitalWrite(MOTOR1_PIN2, HIGH);
    ledcWrite(PWMChannel_1, pwm);
    Serial.println("Hatra");
    break;
  case 3:
    // fek
    digitalWrite(MOTOR1_PIN1, HIGH);
    digitalWrite(MOTOR1_PIN2, HIGH);
    ledcWrite(PWMChannel_1, pwm);
    Serial.println("Megall");
    break;
  default:
    break;
  }
}
//-------------------------------------------------------------------------------------------------------------------- Set right PWM ----------------------------
void Set_right_motor(int direcrion,int pwm)
{
  switch (direcrion)
  {
  case 0:
    // megall
  digitalWrite(MOTOR2_PIN1, LOW);
  digitalWrite(MOTOR2_PIN2, LOW);
  ledcWrite(PWMChannel_2, pwm);
  Serial.println("Megall");
    break;
  case 1:
    // elorre
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, LOW);  
    ledcWrite(PWMChannel_2, pwm);
  Serial.println("Elorre");
    break;
  case 2:
    // hatra
    digitalWrite(MOTOR2_PIN1, LOW);
    digitalWrite(MOTOR2_PIN2, HIGH);
    ledcWrite(PWMChannel_2, pwm);
    Serial.println("Hatra");
    break;
  case 3:
    // fek
    digitalWrite(MOTOR2_PIN1, HIGH);
    digitalWrite(MOTOR2_PIN2, HIGH);
    ledcWrite(PWMChannel_2, pwm);
    Serial.println("Megall");
    break;
  default:
    break;
  }
}
//-------------------------------------------------------------------------------------------------------------------- Set red led ----------------------------
void Set_Red_LED(bool valure)
{
  digitalWrite(LED_PIN1, valure);
}
//-------------------------------------------------------------------------------------------------------------------- Set yellow led ----------------------------
void Set_Yelow_LED(bool valure)
{
  digitalWrite(LED_PIN2, valure);
}
//-------------------------------------------------------------------------------------------------------------------- Set green led ----------------------------
void Set_Green_LED(bool valure)
{
  digitalWrite(LED_PIN3, valure);
}
//--------------------------------------------------------------------------------------------------------------------send_data----------------------------
void send_data(int Id, int distance1, int distance2, unsigned int* sensorValues, int leftMotorPwm, int rightMotorPwm , int errorrr) {

  if (WiFi.status() == WL_CONNECTED) { //Check WiFi connection status

    HTTPClient http;
    String URL = "http://crybe.co/platooning/distance2.php?";
    URL += "Id=" + String(Id);
    URL += "&Distance1=" + String(distance1);
    URL += "&Distance2=" + String(distance2);
    URL += "&IR1=" + String(random(100));
    URL += "&IR2=" + String(random(100));
    URL += "&IR3=" + String(random(100));
    URL += "&IR4=" + String(random(100));
    URL += "&IR5=" + String(random(100));
    URL += "&IR6=" + String(random(100));
    URL += "&IR7=" + String(random(100));
    URL += "&IR8=" + String(random(100));
    URL += "&LeftMotorPwm=" + String(leftMotorPwm);
    URL += "&RightMotorPwm=" + String(rightMotorPwm);
    URL += "&Error=" + String(errorrr);


    /* byte URL[] = "http://crybe.co/platooning/distance2.php?Id=273";
      URL+="Id="+byte[](Id);
      URL+="&Distance1="+byte[](Distance1);
      URL+="&Distance2="+byte[](Distance2);
      URL+="&PositionX="+byte[](PositionX);
      URL+="&PositionY="+byte[](PositionY);
      URL+="&Ir="+byte[](Ir);*/


    Serial.println(URL);
    http.begin(URL);

    int httpCode = http.GET();

    Serial.print("httpcode: ");
    Serial.println(httpCode);

    if (httpCode == HTTP_CODE_OK) {
      Serial.print("HTTP response code ");
      Serial.println(httpCode);
      String response = http.getString();
      Serial.println(response);
    } else {
      Serial.println("Error in HTTP request");
    }

    http.end();

  } else {
    Serial.println("Error in WiFi connection");
  }
  //------------------------------------------------------------------------------------------------------------------------------------------------
}
