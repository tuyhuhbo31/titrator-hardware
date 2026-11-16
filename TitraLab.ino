#include <Arduino.h>
#include <Wire.h>
#include <iarduino_I2C_pH.h>
#include <math.h>

constexpr int SDA_PIN=21, SCL_PIN=22, PH_ADDRESS=0x09;
constexpr int HOME_PIN=34, ENA_PIN=27, STEP_PIN=25, DIR_PIN=26;
constexpr int STIR_IN1=18, STIR_IN2=19, STIR_EN=14;
constexpr int STIR_PWM_FREQ=30000, STIR_PWM_BITS=8, STIR_PWM_CHANNEL=0;
int stirPwm=0;
void setStirrer(int pwm){
 stirPwm=constrain(pwm,0,255);
 if(stirPwm==0){
   ledcWrite(STIR_EN,0);
   digitalWrite(STIR_IN1,LOW);digitalWrite(STIR_IN2,LOW);
 }else{
   digitalWrite(STIR_IN1,LOW);digitalWrite(STIR_IN2,HIGH);
   ledcWrite(STIR_EN,stirPwm);
 }
 Serial.print("STIR,");Serial.println(stirPwm);
}
constexpr double ML_PER_STEP=2.3267/5000.0;
constexpr long MAX_FORWARD_STEPS=40000; // Conservative software limit; verify mechanical range before use
constexpr unsigned long STEP_HALF_US=10000;
constexpr unsigned long PH_INTERVAL_MS=1000;
iarduino_I2C_pH ph(PH_ADDRESS);
bool phConnected=false, moving=false, pulseHigh=false, positionReferenced=false;
long positionSteps=0, targetSteps=0, experimentStartSteps=0;
unsigned long lastEdgeUs=0, lastPhMs=0;
float lastPh=NAN;
String inputLine="";

void sendStatus(){
 Serial.print("STATUS,"); Serial.print(moving?"MOVING":"IDLE"); Serial.print(',');
 Serial.print(positionSteps); Serial.print(','); Serial.print((positionSteps-experimentStartSteps)*ML_PER_STEP,5);
 Serial.print(','); if(isfinite(lastPh)) Serial.print(lastPh,3); else Serial.print("nan");
 Serial.print(','); Serial.println(positionReferenced?"REFERENCED":"UNREFERENCED");
}
void stopMotor(){moving=false; pulseHigh=false; digitalWrite(STEP_PIN,LOW); targetSteps=positionSteps; Serial.println("STOPPED"); sendStatus();}
void processCommand(String line){
 line.trim(); line.toUpperCase();
 if(line=="STOP"){stopMotor();return;}
 if(line=="STIR_STOP"){setStirrer(0);return;}
 if(line.startsWith("STIR=")){
   String value=line.substring(5);value.trim();
   if(value.isEmpty()){Serial.println("ERROR,INVALID_STIR");return;}
   for(unsigned int i=0;i<value.length();i++)if(!isDigit(value[i])){Serial.println("ERROR,INVALID_STIR");return;}
   long pwm=value.toInt();
   if(pwm<0||pwm>255){Serial.println("ERROR,STIR_RANGE");return;}
   setStirrer((int)pwm);return;
 }
 if(line=="STATUS"){sendStatus();return;}
 if(line=="READ"){sendStatus();return;}
 if(line=="ZERO"){
   if(moving){Serial.println("ERROR,BUSY");return;}
   experimentStartSteps=positionSteps; Serial.println("ZEROED");sendStatus();return;
 }
 if(line=="REFERENCE"){
   // Only allow a user-confirmed physical starting position; this is not automatic homing.
   if(moving){Serial.println("ERROR,BUSY");return;}
   positionSteps=0;targetSteps=0;experimentStartSteps=0;positionReferenced=true;
   Serial.println("REFERENCED");sendStatus();return;
 }
 if(line.startsWith("DOSE=")){
   if(moving){Serial.println("ERROR,BUSY");return;}
   if(!positionReferenced){Serial.println("ERROR,REFERENCE_REQUIRED");return;}
   String amount=line.substring(5); amount.trim();
   if(amount.length()==0){Serial.println("ERROR,INVALID_DOSE");return;}
   for(unsigned int i=0;i<amount.length();i++) if(!isDigit(amount[i]) && amount[i]!='.') {Serial.println("ERROR,INVALID_DOSE");return;}
   double ml=amount.toDouble();
   if(!isfinite(ml)||ml<=0||ml>2.0){Serial.println("ERROR,DOSE_RANGE");return;}
   long n=lround(ml/ML_PER_STEP);
   if(n<1 || positionSteps+n>MAX_FORWARD_STEPS){Serial.println("ERROR,TRAVEL_LIMIT");return;}
   // Home switch is not treated as a forward limit without confirmed wiring/placement.
   targetSteps=positionSteps+n; moving=true; pulseHigh=false;
   digitalWrite(DIR_PIN,HIGH); lastEdgeUs=micros();
   Serial.print("STARTED,");Serial.println(n);return;
 }
 Serial.println("ERROR,UNKNOWN_COMMAND");
}
void pollSerial(){
 while(Serial.available()){
   char c=(char)Serial.read();
   if(c=='\n'||c=='\r'){
     if(inputLine.length()) {processCommand(inputLine);inputLine="";}
   }else if(inputLine.length()<80){inputLine+=c;}
   else {inputLine="";Serial.println("ERROR,LINE_TOO_LONG");}
 }
}
void tickMotor(){
 if(!moving)return;
 unsigned long now=micros();
 if((unsigned long)(now-lastEdgeUs)<STEP_HALF_US)return;
 lastEdgeUs=now;
 if(!pulseHigh){
   if(positionSteps>=targetSteps){moving=false;Serial.println("DONE");sendStatus();return;}
   digitalWrite(STEP_PIN,HIGH);pulseHigh=true;
 }else{
   digitalWrite(STEP_PIN,LOW);pulseHigh=false;positionSteps++;
   if(positionSteps>=targetSteps){moving=false;Serial.println("DONE");sendStatus();}
 }
}
void setup(){
 Serial.begin(115200);
 pinMode(STIR_IN1,OUTPUT);pinMode(STIR_IN2,OUTPUT);pinMode(STIR_EN,OUTPUT);
 digitalWrite(STIR_IN1,LOW);digitalWrite(STIR_IN2,LOW);
 if(!ledcAttachChannel(STIR_EN,STIR_PWM_FREQ,STIR_PWM_BITS,STIR_PWM_CHANNEL)){
   Serial.println("ERROR,STIR_PWM_INIT");
 }
 ledcWrite(STIR_EN,0);
 pinMode(ENA_PIN,OUTPUT);pinMode(STEP_PIN,OUTPUT);pinMode(DIR_PIN,OUTPUT);pinMode(HOME_PIN,INPUT);
 digitalWrite(STEP_PIN,LOW);digitalWrite(DIR_PIN,HIGH);digitalWrite(ENA_PIN,LOW);
 Wire.begin(SDA_PIN,SCL_PIN);Wire.setClock(100000);
 phConnected=ph.begin();
 Serial.println(phConnected?"PH_CONNECTED":"PH_NOT_FOUND");
 Serial.println("READY");sendStatus();
}
void loop(){
 pollSerial();tickMotor();
 unsigned long now=millis();
 if(now-lastPhMs>=PH_INTERVAL_MS){
   lastPhMs=now;
   if(!phConnected)phConnected=ph.begin();
   if(phConnected){float value=ph.getPH();if(isfinite(value))lastPh=value;}
   sendStatus();
 }
}
