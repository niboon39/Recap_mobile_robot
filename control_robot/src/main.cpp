#include <Arduino.h>
#include "PinChangeInterrupt.h"
#include "ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

/* Pin Motor  -> L298N  */  
#define L_motor_PWM 6 
#define R_motor_PWM 5 
/* Motor left */  
#define L_motor_DIR1 8 
#define L_motor_DIR2 9 
/* Motor right */
#define R_motor_DIR1 10 
#define R_motor_DIR2 7 

int ENr_A  = 2 ; // Interrupt 0
int ENr_B  = 3 ; // Interrupt 1 
int ENl_A  = A1 ; 
int ENl_B  = A0 ; 

volatile long pulse_l = 0 ;
volatile long pulse_r = 0 ; 
volatile long time_old = 0 ;

void Count_l (){
  if (digitalRead(ENl_B) == 0 ){  // In board phaseA : B2
    if (digitalRead(ENl_A) == 0){ // In board phaseA : A2
      pulse_l ++ ; 
    }else{
      pulse_l -- ; 
    }
  }
}

void Count_r (){
  if (digitalRead(ENr_B) == 0 ){  // In board phaseA : B1
    if (digitalRead(ENr_A) == 0){ // In board phaseA : A1 
      pulse_r ++ ; 
    }else{
      pulse_r -- ; 
    }
  }
}

void Init_pin_var(){
  digitalWrite(L_motor_DIR1 , 0);digitalWrite(L_motor_DIR2 , 0) ;
  digitalWrite(R_motor_DIR1 , 0);digitalWrite(R_motor_DIR2 , 0) ;
  analogWrite(L_motor_PWM , 0 ) ; analogWrite(R_motor_PWM , 0 ) ; 
  pulse_r = 0 ; 
  pulse_l = 0 ;
  time_old = 0 ; 
}


float PPR = 210 ; // 210 counts per main shaft revolution

/* Set P , I  */
float kp  = 0.5  ; 
float ki  = 0.05 ; 

/*  Left Wheel Control */
unsigned long curTimeL , prevTimeL , diffTimeL ; 
long curTickL , prevTickL , diffTickL ; 
double errorL , setRPML , lastErrorL , sumErrorL ; 
double controlL_outL ; 
double measuredRPML ; 

/* Right Wheel Control */
unsigned long curTimeR , prevTimeR , diffTimeR ; 
long curTickR , prevTickR , diffTickR ; 
double errorR , setRPMR , lastErrorR , sumErrorR ; 
double controlR_outR ;
double measuredRPMR ; 

/* OUTPUT PI Control */
double desiredRPMR , desiredRPML ; 

/* Message ros setup */ 
std_msgs::Int32 rticks_msg ;      // Publish Right ticks 
std_msgs::Int32 lticks_msg ;      // Publish left ticks 
std_msgs::Float32 rpm_right_msg ; // Publish rpm motor right 
std_msgs::Float32 rpm_left_msg ;  // Publish rpm motor left 

ros::Publisher rticks_pub("/tick_wheel_right" , &rticks_msg); 
ros::Publisher lticks_pub("/tick_wheel_left" , &lticks_msg); 
ros::Publisher rpm_right_pub("/rpm_right" , &rpm_right_msg); 
ros::Publisher rpm_left_pub("/rpm_left" , &rpm_left_msg); 


void turnWheelL (float setpoint , long inTick){
  unsigned int l_pwm ;
  curTickL = inTick ; // current encoder l 
  curTimeL = millis();
  setRPML = setpoint ; 

  diffTickL = curTickL - prevTickL ;
  diffTimeL = curTimeL - prevTimeL ;  

  measuredRPML = ( ( diffTickL / PPR ) / ( diffTimeL * 0.001 ) ) * 60 ; 
  // Serial.print("measuredRPM L : ");
  // Serial.println(measuredRPML) ; 

  rpm_left_msg.data = measuredRPML ; 

  /* Check setpoint find error */
  if (setRPML >= 0){
    errorL = setRPML -  measuredRPML ;
  }else if (setRPML < 0 ){
    errorL = measuredRPML - setRPML ; 
  }
  sumErrorL += errorL * diffTimeL ; 
  controlL_outL = (kp * errorL) + (ki * sumErrorL) ; 

  /* Update value */
  lastErrorL = errorL ; 
  prevTickL = curTickL ; 
  prevTimeL = curTimeL ; 

  if ( setRPML > 0  ){
    digitalWrite(L_motor_DIR1 , 0) ; 
    digitalWrite(L_motor_DIR2 , 1) ;
    l_pwm = controlL_outL ; 
  }else if (setRPML < 0 ){
    digitalWrite(L_motor_DIR1 , 1) ;
    digitalWrite(L_motor_DIR2 , 0) ;
    l_pwm = controlL_outL ; 
  }else{
    digitalWrite(L_motor_DIR1 , 0) ;
    digitalWrite(L_motor_DIR2 , 0) ;
    l_pwm = 0 ; 
  }
  if (l_pwm < 0 ){
    l_pwm = 0 ; 
  }
  analogWrite(L_motor_PWM , l_pwm); 
}

void turnWheelR (float setpoint , long inTick){
  unsigned int r_pwm ; 
  curTickR = inTick ; 
  curTimeR = millis() ; 
  setRPMR = setpoint ; 

  diffTickR = curTickR - prevTickR ; 
  diffTimeR = curTimeR - prevTimeR ; 

  measuredRPMR = ( (diffTickR / PPR) / (diffTimeR / 0.001) ) * 60 ; 

  rpm_right_msg.data = measuredRPMR ; 

  /* Check setpoint find error */
  if ( setpoint > 0  ){
    errorR = setRPMR - measuredRPMR ; 
  }else if ( setpoint < 0  ){
    errorR = measuredRPMR - setRPMR ; 
  }

  sumErrorR += errorR  * diffTimeR ; 
  controlR_outR = (kp*errorR) + (ki*sumErrorR) ; 

  /* Update value */
  lastErrorR = errorR ; 
  prevTickR = curTickR ; 
  prevTimeR = curTimeR ; 

  if ( setRPMR > 0  ){ // forward 
    digitalWrite(R_motor_DIR1 , 0) ; 
    digitalWrite(R_motor_DIR2 , 1) ;
    r_pwm = controlR_outR ;  
  }else if (setRPMR < 0 ){ // backward 
    digitalWrite(R_motor_DIR1 , 1) ;
    digitalWrite(R_motor_DIR2 , 0) ;
    r_pwm = controlR_outR ; 
  }else{
    digitalWrite(R_motor_DIR1 , 0) ;
    digitalWrite(R_motor_DIR2 , 0) ;
    r_pwm = 0 ; 
  }
  analogWrite(R_motor_PWM , r_pwm) ; 
}

void leftwheelcb (const std_msgs::Float32 &wheel_power){
  desiredRPML = wheel_power.data ; 
}

void rightwheelcb (const std_msgs::Float32 &wheel_power){
  desiredRPMR = wheel_power.data ; 
}


ros::Subscriber<std_msgs::Float32> sub_right("/wheel_power_right" , &rightwheelcb);
ros::Subscriber<std_msgs::Float32> sub_left("/wheel_power_left" , &leftwheelcb);

void debug_encoder () {
  Serial.print("Pluse_L , Pulse_R : ") ; 
  Serial.print(pulse_l) ;Serial.print(" ") ;Serial.println(pulse_r) ;  
  delay(100) ; 
}

void setup() {
  Serial.begin(9600) ; 
  /* Set pin Motor  */
  pinMode(L_motor_PWM  , OUTPUT) ; 
  pinMode(R_motor_PWM  , OUTPUT) ; 
  pinMode(L_motor_DIR1 , OUTPUT) ; 
  pinMode(L_motor_DIR2 , OUTPUT) ;
  pinMode(R_motor_DIR1 , OUTPUT) ;
  pinMode(R_motor_DIR2 , OUTPUT) ;  

  /* Init Pin & Var */
  Init_pin_var() ; 

  /* Set pin Encoder */
  pinMode(ENr_A , INPUT); pinMode(ENr_B , INPUT) ; 
  pinMode(ENl_A , INPUT); pinMode(ENl_B , INPUT) ; 

  attachInterrupt(digitalPinToInterrupt(ENr_A) , Count_r , CHANGE) ; 
  attachPCINT(digitalPinToPCINT(ENl_A) , Count_l , CHANGE) ; 

  /* ROS setup */
  nh.initNode();
  Serial.begin(57600);
  nh.subscribe(sub_right) ; 
  nh.subscribe(sub_left) ; 
  nh.advertise(rpm_left_pub) ; 
  nh.advertise(rpm_right_pub) ; 
  nh.advertise(rticks_pub) ; 
  nh.advertise(lticks_pub) ; 
  delay(20) ; 

}

void loop() {
  turnWheelR(desiredRPMR , pulse_r) ; 
  turnWheelL(desiredRPML , pulse_l) ; 

  if ( millis() - time_old >= 100 ){ // update every 1 sec 
    rticks_msg.data = pulse_r ; 
    lticks_msg.data = pulse_l ; 

    /* Publisher value of encoder */
    rticks_pub.publish(&rticks_msg) ; 
    lticks_pub.publish(&lticks_msg);
    /* Publisher value RPM of control */
    rpm_left_pub.publish(&rpm_left_msg) ; 
    rpm_right_pub.publish(&rpm_left_msg) ; 

    time_old = millis() ; 
  }

  nh.spinOnce() ; 


}