#include<Wire.h>
#include <SparkFun_MicroPressure.h>
#include <SensirionI2CSdp.h>
#include "NextionDisplay.h"

NexHotspot page0Menu = NexHotspot(0,1, "m0");
NexHotspot Start = NexHotspot(0,3, "m1");
NexNumber page0press = NexNumber(0,2,"n0");

NexText modeAPAP = NexText(1,5,"t0");
NexNumber minPress = NexNumber(1,6,"n0");
NexNumber maxPress = NexNumber(1,7,"n1");
NexHotspot APAPIncre = NexHotspot(1,2, "m4");
NexHotspot APAPDecre = NexHotspot(1,3, "m5");

NexText modeCPAP = NexText(2,5,"t1");
NexNumber SetPress = NexNumber(2,6,"n0");
NexHotspot CPAPIncre = NexHotspot(2,2, "m3");
NexHotspot CPAPDecre = NexHotspot(2,3, "m4");

NexHotspot page7Menu = NexHotspot(7,1, "m0");
NexHotspot Stop = NexHotspot(7,2, "m1");
NexNumber page7press = NexNumber(7,3,"n0");


SoftwareSerial HMISerial(2,3);
SensirionI2CSdp sdp;
SparkFun_MicroPressure mpr;


  
  float flowConstant;
  float atmosPressure;
  float kP, kI, kD;
   float setPoint = 4;
  unsigned long previousTime, currentTime, previousTimePressureFix;
   float elapsedTime;
   float rateError, cumError;
   float error, lastError;
   float Kv = 0.03087;
   int modes, increase, Select;
  
NexTouch *nex_listen_list[] = 
{
    &page0Menu,
 &Start,
&page0press,
 &modeAPAP ,
&minPress,
&maxPress,
&APAPIncre,
&APAPDecre,

 &modeCPAP,
&SetPress,
&CPAPIncre,
 &CPAPDecre,

 &page7Menu, 
&Stop,
&page7press, 
};


 float computePID( float inp) {
   currentTime = millis();
   elapsedTime = ( float) ( currentTime -  previousTime);
   error =  setPoint - inp;
   cumError +=   error *  elapsedTime;
   rateError = ( error -  lastError) /  elapsedTime;

   float output =  error *  kP +  cumError *  kI +  rateError *  kD;
  if(output > 300) {
    output = 300                                     ;
  }
  if(output <0) {
    output = 0;
  }
  
   previousTime =  currentTime;
  return output;

  
  }

 float pressureRead( float value) {
   float gauge = (value -  atmosPressure)*1000;
  return gauge;
  }

 float flowCalculation( float dp) {
   float flow;
  if(dp < 0){
    dp = -1*(dp);
   flow =  Kv * sqrt((dp/0.00103));
   return -(flow);
  }
  else {
   flow =  Kv * sqrt((dp/0.00103));
   return flow;
  }
  }

void PIDConstantFix(float input) {

  if(input<=4) {
     kP = 1.5;
     kI = 0.012;
     kD = 0;
  }
   else if((input > 4)&& (input <=4.5)) {
     kP = 1.5;
     kI = 0.012;
     kD = 0;
  }
  else if((input > 4.5)&& (input <=5)) {
     kP = 1.5;
     kI = 0.012;
     kD = 0;
  }
  else if((input > 5)&& (input <=5.5)) {
     kP = 1.5;
     kI = 0.012;
     kD = 0;
  }
  else if((input > 5.5)&& (input <=6)) {
     kP = 1.5;
     kI = 0.012;
     kD = 0;
  }else if((input > 6)&& (input <=6.5)) {
     kP = 1.5;
     kI = 0.01;
     kD = 0;
  }
  else if((input > 6.5)&& (input <=7) ){
     kP = 1.5;
     kI = 0.011;
     kD = 0;
  }
  else if((input > 7)&& (input <=7.5)){
     kP = 1.5;
     kI = 0.01;
     kD = 0;
  }
  else if((input > 7.5)&& (input <=8)){
     kP = 1.5;
     kI = 0.012;
     kD = 0;
  }
  else if((input > 8)&& (input <=8.5)){
     kP = 1.5;
     kI = 0.012;
     kD = 0;
  }
  else if((input > 8.5)&& (input <=9)){
     kP = 1.5;
     kI = 0.012;
     kD = 0;
  }
  else if((input > 9)&& (input <=9.5)) {
     kP = 1.5;
     kI = 0.009;
     kD = 0;
  }
  else if((input > 9.5)&& (input <=10)){
     kP = 1.35;
     kI = 0.006;
  }
  else if((input > 10)&& (input <=10.5)){
     kP = 1.35;
     kI = 0.006;
     kD = 0;
  }
  else if((input > 10.5)&& (input <=11)) {
     kP = 1.35;
     kI = 0.006;
  }
  else if((input > 11)&& (input <=11.5) ){
     kP = 1.35;
     kI = 0.0008;
     kD = 0;
  }
  else if((input > 11.5)&& (input <=12)) {
     kP = 1.35;
     kI = 0.0008;
     kD = 0;
  }
  else if((input > 12)&& (input <=12.5)) {
     kP = 1.35;
     kI = 0.0008;
     kD = 0;
  }
  else if((input > 12.5)&& (input <=13)) {
     kP = 1.35;
     kI = 0.0008;
     kD = 0;
  }
  else if((input > 13)&& (input <=13.5)) {
     kP = 1.35;
     kI = 0.0002;
     kD = 0;
  }
  else if((input > 13.5)&& (input <=14)) {
     kP = 1.35;
     kI = 0.0002;
  }
  else if((input > 14.5)&& (input <=15)) {
     kP = 1.35;
     kI = 0.0002;
     kD = 0;
  }
  else if((input > 15)&& (input <=15.5)) {
     kP = 1.35;
     kI = 0.0002;
  }
  else if((input > 15.5)&& (input <=16)) {
     kP = 1.25;
     kI = 0.0002;
     kD = 0;
  }
  else if((input > 16)&& (input <=16.5)) {
     kP = 1.5;
     kI = 0.0002;
     kD = 0;
  }
  else if((input > 16.5)&& (input <=17)) {
     kP = 1.25;
     kI = 0.0002;
     kD = 0;
    
  }
  else if((input > 17)&& (input <=17.5)) {
     kP = 1.25;
     kI = 0.0002;
     kD = 0;
  }
  else if((input > 17.5)&& (input <=18)) {
     kP = 1.25;
     kI = 0.0002;
     kD = 0;
  }
  else if((input > 18)&& (input <=18.5)) {
     kP = 1.25;
     kI = 0.0002;
     kD = 0;
  }
  else if((input > 18.5)&& (input <=19)) {
     kP = 1.25;
     kI = 0.0002;
     kD = 0;
  }
  else{
     kP = 1.25;
     kI = 0.0002;
     kD = 0;
  }
  
  
}
bool process = false;
void analogWrite25k(int pin, int value)
{
    switch (pin) {
        case 9:
            OCR1A = value;
            break;
        case 10:
            OCR1B = value;
            break;
        default:
            // no other pin will work
            break;
    }
}

void Menu(void *ptr) {
  modes=2;
}

void modeApap(void *ptr) {
  modes=0;
}

void ModeCPAP(void *ptr) {
  modes=1;
}
void minpress(void *ptr) {
  
}

void maxpress(void *ptr) {
  
}
void setPress(void *ptr) {
  Select =1;
}

void CPAPIncres(void *ptr) {
  increase++;
  if(increase >21) {
    increase =21;
  }
}


void CPAPDecres(void *ptr) {
  increase--;
  if(increase <0) {
    increase =0;
  }
}

void start(void *ptr) {
  process =  true;
}

void stoper(void *ptr) {
  process =  false;
}

void APAPincres(void *ptr) {
  
}
void APAPdecres(void *ptr) {
  
}

void APAPmode(){
  
if(process) {

  float currentPressure = pressureRead(mpr.readPressure(BAR));
  PIDConstantFix((float) setPoint);
  float output = computePID(( float)currentPressure);
  analogWrite25k(9, output);
  
  
 
  uint16_t error;
  char errorMessage[256];


    // Read Measurement
  float differentialPressure;
  float temperature;
  bool r = false;
  error = sdp.readMeasurement(differentialPressure, temperature);

  if (error) {
     Serial.print("Error trying to execute readMeasurement(): ");
     errorToString(error, errorMessage, 256);
     Serial.println(errorMessage);
    } else {
        Serial.print("DifferentialPressure[Pa]:");
        Serial.print(differentialPressure);
        Serial.print("\t");
        Serial.print("Temperature[°C]:");
        Serial.print(temperature);
        Serial.println();
        r=true;
    }
  
  

  if(r) {
  float flow = flowCalculation(differentialPressure);
  Serial.print("Flow LPM = ");
  Serial.println(flow);
//  float MeterCube = flow*0.000016666666666666667;
//  float areaCross = 3.14*81;
//  float currentPressure = ((0.5 *1.229 * (MeterCube/areaCross)* (MeterCube/areaCross)) + (1.229 * 9.8* 1.4))* 0.00001; 
  
   Serial.print("out = ");
  Serial.println(output);
  Serial.print("current pressure = ");
  Serial.println(currentPressure);
  
  if((flow < 5.000) && (flow > -2.00)) 
  {
   
     if(millis() -  previousTimePressureFix >= 3000) {
        previousTimePressureFix = millis();
       Serial.println("Time Check");
       nexLoop(nex_listen_list);
    if((currentPressure <  setPoint + 0.3) &&  (currentPressure >  setPoint - 0.3)) { setPoint =  setPoint + 0.5;};
    
    if( setPoint > 20.00) {
       setPoint = 20;
    }
    Serial.print("SetPoint = ");
    Serial.println( setPoint);

    page0press.setValue((int) setPoint);
    page7press.setValue((int) setPoint);
   
  }
  
  
  
  }
  Serial.print("SetPoint = ");
  Serial.println( setPoint);
  }
  
}
}


void CPAPMode() {
  if(process) {
    uint32_t number = 4;
    SetPress.getValue(&number);
    setPoint = (float) number;
    page0press.setValue((int)number);
    page7press.setValue((int)number);
    float currentPressure = pressureRead(mpr.readPressure(BAR));
    PIDConstantFix((float) setPoint);
    float output = computePID(( float)currentPressure);
    analogWrite25k(9, output);
  }
}
void fullFunctions() {
  page0Menu.attachPop(Menu, &page0Menu);
  Start.attachPop(start, &Start);
  
  modeAPAP.attachPop(modeApap, &modeAPAP);
  minPress.attachPop(minpress, &minPress);
  maxPress.attachPop(maxpress, &maxPress);
  APAPIncre.attachPop(APAPincres, &APAPIncre);
  APAPDecre.attachPop(APAPdecres, &APAPDecre);
  modeCPAP.attachPop(ModeCPAP, &modeCPAP);
  SetPress.attachPop(setPress, &SetPress);
  CPAPIncre.attachPop(CPAPIncres, &CPAPIncre);
  CPAPDecre.attachPop(CPAPDecres, &CPAPDecre);
  page7Menu.attachPop(Menu, &page7Menu); 
  Stop.attachPop(stoper, &Stop);
  
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  Wire.begin();
  Serial.println("Startted");
  nexInit();
  // Configure Timer 1 for PWM @ 25 kHz.
    TCCR1A = 0;           // undo the configuration done by...
    TCCR1B = 0;           // ...the Arduino core library
    TCNT1  = 0;           // reset timer
    TCCR1A = _BV(COM1A1)  // non-inverted PWM on ch. A
           | _BV(COM1B1)  // same on ch; B
           | _BV(WGM11);  // mode 10: ph. correct PWM, TOP = ICR1
    TCCR1B = _BV(WGM13)   // ditto
           | _BV(CS10);   // prescaler = 1
    ICR1   = 320;         // TOP = 320

  
  sdp.begin(Wire, SDP8XX_I2C_ADDRESS_0);
  uint16_t error;
  char errorMessage[256];
  uint32_t productNumber;
  uint8_t serialNumber[8]; 
  uint8_t serialNumberSize = 8;

  sdp.stopContinuousMeasurement();

  error= sdp.readProductIdentifier(productNumber, serialNumber, serialNumberSize);

  if(error) {
    Serial.print("Error trying to execute : ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    }   else {
                Serial.print("ProductNumber:");
                Serial.print(productNumber);
                Serial.print("\t");
                Serial.print("SerialNumber:");
                Serial.print("0x");
                for (size_t i = 0; i < serialNumberSize; i++) {
                    Serial.print(serialNumber[i], HEX);
                }
                Serial.println();
            }

    error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();

    if (error) {
        Serial.print(
            "Error trying to execute "
            "startContinuousMeasurementWithDiffPressureTCompAndAveraging(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }
  
  if(!mpr.begin()){
    Serial.println("Not working");
    while(1);
  }
  //analogWrite25k(9, 35);
  delay(3000);
  atmosPressure= mpr.readPressure(BAR);
  
  delay(3000);
  fullFunctions();
  
}

void loop() {
  // put your main code here, to run repeatedly:.
  
   nexLoop(nex_listen_list);
   if(modes == 0) {
   APAPmode();
   }
 
  
  else if(modes ==1) {
    CPAPMode();
  }

  else {
     analogWrite25k(9, 0);
     
  }
}

 
