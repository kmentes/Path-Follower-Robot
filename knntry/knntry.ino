#include <ECE3.h>


const int left_nslp_pin  = 31;  // nslp = not sleep --> set high --> digitalWrite
const int left_dir_pin   = 29;  // control direction of motor --> digitalWrite
const int left_pwm_pin   = 40;  // use to control motor speed --> analogWrite
const int right_nslp_pin = 11;
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;
const int speed = 72;

const double kP = 11;
const double kD = 3;
//12,3

double speedAdjust;
double pastEr;
double error;
int previous;
int pastSpeed;
int lines;
double w[8] = {-1.25, -.70, -.35, -.10, .10, .35, .70, 1.25};
bool flag;
unsigned long StartTime;


uint16_t sensorValues[8];

void setup() {
  setUpMotorPins();
  ECE3_Init();
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Simple KNN");

  Serial.print("Adding examples to myKNN ... ");
  Serial.println();

  // add examples to KNN
  
  // you can also print the counts by class
  //  Serial.print("\tmyKNN.getCountByClass(5) = ");
  //  Serial.println(myKNN.getCountByClass(5)); // expect 2

  // classify the input
  Serial.println("Classifying input ...");
  previous = 0;
  error = 0;
  pastEr= 0;
  pastSpeed = 0;
  speedAdjust = 0;
  lines = 0;
  flag = false;
  StartTime = millis();
}

void loop() {
  // put your main code here, to run repeatedly: 
  unsigned long CurrentTime = millis();
  unsigned long ElapsedTime = CurrentTime - StartTime;
  ECE3_read_IR(sensorValues);
  float input[] = {sensorValues[0],sensorValues[1],sensorValues[2],sensorValues[3],sensorValues[4],sensorValues[5],sensorValues[6],sensorValues[7],sensorValues[8]};
  
   double mini = sensorValues[0];
  for(int i = 0; i < 8; i++){
    if(mini>sensorValues[i]){
      mini = sensorValues[i];
    }
  }
  for(int i = 0; i < 8; i++){
    
    sensorValues[i] = sensorValues[i] - mini;
  }
  double maxi = sensorValues[0];
  for(int i = 0; i < 8; i++){
    if(maxi < sensorValues[i]){
      maxi = sensorValues[i];
    }
  }
  for(int i = 0; i < 8; i++){
   if(sensorValues[i] != 0) sensorValues[i] = 10.0*sensorValues[i]/maxi;
  }
 

  error = sensorValues[0]*w[0] + sensorValues[1]*w[1] + sensorValues[2]*w[2] + sensorValues[3]*w[3] + sensorValues[4]*w[4] + sensorValues[5]*w[5] + 
                  sensorValues[6]*w[6] + sensorValues[7]*w[7]; 
                  
  double speedAdjust = kP * error + kD * (error - pastEr);
  
  //10 -> -10 range
  //speedAdjust rang -40 -> 40
  
 
  //Serial.print("\tclassification = ");
  //Serial.println(classification);
  double simpleSum = sensorValues[0] + sensorValues[1] +sensorValues[2] +sensorValues[3] +sensorValues[4] +sensorValues[5] +sensorValues[6] +sensorValues[7];
  if(simpleSum >= 40 && ElapsedTime >11000){
        analogWrite(left_pwm_pin,0);
        analogWrite(right_pwm_pin,0);
        while(1);
      //use time to stop
  }
  if(simpleSum >= 40 && ElapsedTime >4000){
        
        digitalWrite(left_dir_pin, HIGH);
        analogWrite(left_pwm_pin, 120);
        analogWrite(right_pwm_pin, 120);
        delay(600);
        digitalWrite(left_dir_pin, LOW);
      //use time to stop
  }
  if(simpleSum > 40){
    speedAdjust = pastSpeed;
  }
  else if(ElapsedTime<10000 && ElapsedTime > 9500 && speedAdjust > 5){
    analogWrite(right_pwm_pin, (speed/4 + speedAdjust));
    analogWrite(left_pwm_pin, 0.6*(speed/4 - speedAdjust));
  }
  //else if(speedAdjust <-2){
     //analogWrite(right_pwm_pin, speed + speedAdjust);
     //analogWrite(left_pwm_pin, 0.6 * (speed - speedAdjust));
  //}
  //else if(speedAdjust > 2){
     //analogWrite(right_pwm_pin, (speed + speedAdjust));
     //analogWrite(left_pwm_pin, 0.6 * (speed - speedAdjust));
  //}
  else if(ElapsedTime > 11000 && simpleSum > 40){
    analogWrite(right_pwm_pin, 0);
    analogWrite(left_pwm_pin, 0);
    while(1);
  }
  else{
     analogWrite(right_pwm_pin, speed + speedAdjust);
     analogWrite(left_pwm_pin, speed - speedAdjust);
  }
 

  
  pastEr = error;
  pastSpeed = speedAdjust;
}

void setUpMotorPins()
{
    pinMode(left_nslp_pin, OUTPUT);
    pinMode(left_dir_pin, OUTPUT);
    pinMode(left_pwm_pin, OUTPUT);

    pinMode(right_nslp_pin, OUTPUT);
    pinMode(right_dir_pin, OUTPUT);
    pinMode(right_pwm_pin, OUTPUT);

    digitalWrite(left_nslp_pin, HIGH);
    digitalWrite(left_dir_pin, LOW);

    digitalWrite(right_nslp_pin, HIGH);
    digitalWrite(right_dir_pin, LOW);
}
