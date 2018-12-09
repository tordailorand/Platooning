#include <WiFi.h>

#include <HTTPClient.h>
#include "Adafruit_VL53L0X.h"
#include <QTRSensors.h>

#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31

#define SHT_SENSOR1 23
#define SHT_SENSOR2 17

#define QTRRC_EN 13


int Id = 2;


//const char* SsId = "Crybe";
//const char* Password =  "Crybe123456";

//const char* SsId = "Internet";
//const char* Password =  "";


int NumberOfQtrSensors = 8; 

const char* SsId = "Kucko4";
const char* Password =  "kkuucckkoo";

Adafruit_VL53L0X DistanceSensor1 = Adafruit_VL53L0X();
Adafruit_VL53L0X DistanceSensor2 = Adafruit_VL53L0X();



QTRSensorsRC qtrrc((unsigned char[]) {1,2,3,4,5,6,7,8},NumberOfQtrSensors); //


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

  pinMode(SHT_SENSOR1, OUTPUT);
  pinMode(SHT_SENSOR2, OUTPUT);
  pinMode(QTRRC_EN, OUTPUT);  //Enable pin of QTRRC
  
  digitalWrite(SHT_SENSOR1, LOW);
  digitalWrite(SHT_SENSOR2, LOW);
  digitalWrite(QTRRC_EN, HIGH); 
  
  //Set_distance_sensors();
  Set_wifi(SsId,Password); 
}
//----------------------------------------------------------------------------------------------------Loop-------------------------------------------- 
void loop() {
  
    distance1 = random(300);
    distance2 = random(300);
    leftMotorPwm = random(300);
    rightMotorPwm = random(300);
    errorrr = random(300);
    
  
  
  
  send_data(Id,distance1,distance2,SensorValues,leftMotorPwm,rightMotorPwm,errorrr); 
  delay(100);
}
//----------------------------------------------------------------------------------------------------Set Wifi-------------------------------------------- 
void Set_wifi(const char* SsId, const char* Password){  
  WiFi.begin(SsId, Password); 
    while (WiFi.status() != WL_CONNECTED) { 
      delay(1000);
      Serial.println("Connecting to WiFi..");
    }
  Serial.println("Connected to the WiFi network");
}
//-------------------------------------------------------------------------------------------------------Set distance sensors-----------------------------------------
void Set_distance_sensors(){ 

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

  // initing LOX1
  if(!DistanceSensor1.begin(SENSOR1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    //while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_SENSOR2, HIGH);
  delay(10);

  //initing LOX2
  if(!DistanceSensor2.begin(SENSOR2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    //while(1);
  }
}

//-------------------------------------------------------------------------------------------------------------measure_distance 1----------------------------------- 
int Get_distance1(){                          
 
  VL53L0X_RangingMeasurementData_t MeasureDistance1;
  Serial.print("Reading a measurement... ");
  DistanceSensor1.rangingTest(&MeasureDistance1, false); // pass in 'true' to get debug data printout!
  
  if (MeasureDistance1.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("distance1 (mm): "); 
    Serial.println(MeasureDistance1.RangeMilliMeter);
  } else {
    Serial.println("distance 1 out of range ");
  }
  
  return MeasureDistance1.RangeMilliMeter;
}
//--------------------------------------------------------------------------------------------------------------measure_distance 2----------------------------------
int Get_distance2(){
  
  VL53L0X_RangingMeasurementData_t MeasureDistance2;
  Serial.print("Reading a measurement... ");
  DistanceSensor2.rangingTest(&MeasureDistance2, false); // pass in 'true' to get debug data printout!

  if (MeasureDistance2.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("distance2 (mm): "); 
    Serial.println(MeasureDistance2.RangeMilliMeter);
  } else {
    Serial.println("distance 2 out of range ");
  }

  return MeasureDistance2.RangeMilliMeter;
}
//--------------------------------------------------------------------------------------------------------------------read_position_x----------------------------
int Get_position_x(){
  int value = random(100);
  return value;
}
//--------------------------------------------------------------------------------------------------------------------read_position_y----------------------------
int Get_position_y(){
  int value = random(100);
  return value;
}
//--------------------------------------------------------------------------------------------------------------------read_ir----------------------------
int Get_ir_sensor(){

  qtrrc.read(SensorValues);
    
  for (int i = 0; i < NumberOfQtrSensors; i++)
  {
    Serial.print(SensorValues[i]);
    Serial.print(' ');
  }
  
  int value = random(100);
  return value;
}
//--------------------------------------------------------------------------------------------------------------------send_data----------------------------
void send_data(int Id, int distance1, int distance2, unsigned int* sensorValues,int leftMotorPwm, int rightMotorPwm ,int errorrr){
  
  if(WiFi.status()== WL_CONNECTED){   //Check WiFi connection status
    
  HTTPClient http; 
   String URL = "http://crybe.co/platooning/distance2.php?";
    URL+="Id="+String(Id);
    URL+="&Distance1="+String(distance1);
    URL+="&Distance2="+String(distance2);
    URL+="&IR1="+String(random(100));
    URL+="&IR2="+String(random(100));
    URL+="&IR3="+String(random(100));
    URL+="&IR4="+String(random(100));
    URL+="&IR5="+String(random(100));
    URL+="&IR6="+String(random(100));
    URL+="&IR7="+String(random(100));
    URL+="&IR8="+String(random(100));
    URL+="&LeftMotorPwm="+String(leftMotorPwm);
    URL+="&RightMotorPwm="+String(rightMotorPwm);
    URL+="&Error="+String(errorrr);
    

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
  
    if(httpCode == HTTP_CODE_OK){
       Serial.print("HTTP response code ");
       Serial.println(httpCode);
       String response = http.getString();
       Serial.println(response);
    }else{
       Serial.println("Error in HTTP request");
    }
 
  http.end();
 
 }else{
    Serial.println("Error in WiFi connection");   
 }
//------------------------------------------------------------------------------------------------------------------------------------------------  
}
