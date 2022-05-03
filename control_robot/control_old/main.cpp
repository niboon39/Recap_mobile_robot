#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include "PinChangeInterrupt.h"
#include <Servo.h>    //servo addition
ros::NodeHandle nh;

int cam_servo_pin = 11;
Servo cam_servo; //servo add
int cur_angle;
int new_angle;
int pos = 0;
#define LED_BUILTIN 13


#define L_MOTOR_PWM 6
#define L_MOTOR_DIR1 8
#define L_MOTOR_DIR2 9
#define R_MOTOR_PWM 5
#define R_MOTOR_DIR1 10
#define R_MOTOR_DIR2 7

void turnWheel( const std_msgs::Float32 &wheel_power,
                unsigned int pwm_pin,
                unsigned int dir1_pin,
                unsigned int dir2_pin) {
    float factor = max(min(wheel_power.data, 1.0f), -1.0f);
    Serial.print(wheel_power.data);
    if( factor >= 0 ) {
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);  
        analogWrite(pwm_pin, (unsigned int)(255 * factor));
    } else {
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
        analogWrite(pwm_pin, (unsigned int)(-255 * (1.0f * factor)));
    }   
}

void rightWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Right");
    //char result[8];
    //dtostrf(wheel_power.data, 6, 2, result); 
    //nh.loginfo(result);

    turnWheel( wheel_power, R_MOTOR_PWM, R_MOTOR_DIR1, R_MOTOR_DIR2 );
    
}

void leftWheelCb( const std_msgs::Float32 &wheel_power ) {
    //nh.loginfo("Wheel Power - Left");
  
    turnWheel( wheel_power, L_MOTOR_PWM, L_MOTOR_DIR1, L_MOTOR_DIR2 );
}

void servoCb( const std_msgs::Int16 &angle)
{
  new_angle = angle.data;
  if(new_angle>(cur_angle-90)){
    
  for (pos = cur_angle;pos<new_angle+90; pos+=1 ){
    cam_servo.write(pos);
    delay(5);
  }
  cur_angle = new_angle+90;
  }
  else if((new_angle<(cur_angle-90))&&(new_angle>=-45)){
  for (pos = cur_angle;pos>new_angle+90; pos-=1 ){
    cam_servo.write(pos);
    delay(5);
  }
  cur_angle = new_angle+90;
  }
}

ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
                                           &leftWheelCb );
ros::Subscriber<std_msgs::Int16> sub_servo("servo", &servoCb);

int r_encoder_pinA = 2; //Interrupt1
int r_encoder_pinB = 3; //Interrupt0
int l_encoder_pinA = A1;
int l_encoder_pinB = A0;

volatile long pulse_l=0;
volatile long pulse_r=0;
unsigned long timeold=0;


//unsigned int pulse_per_rev=20; //Pulse per revolustion of index disc

void counter_l()
{

  if( digitalRead(l_encoder_pinB) == 0 ) {
    if ( digitalRead(l_encoder_pinA) == 0 ) {
      // A fell, B is low
      pulse_l++; // moving reverse
    } else {
      // A rose, B is low
      pulse_l--; // moving forward
    }
  }

}
void counter_r()
{

   if( digitalRead(r_encoder_pinB) == 0 ) {
    if ( digitalRead(r_encoder_pinA) == 0 ) {
      // A fell, B is low
      pulse_r++; // moving reverse
    } else {
      // A rose, B is low
      pulse_r-- ;// moving forward
    }
  }
 
}


std_msgs::Int32 rticks_msg;
std_msgs::Int32 lticks_msg;

ros::Publisher rticks_pub("tick_wheel_right", &rticks_msg);
ros::Publisher lticks_pub("tick_wheel_left", &lticks_msg);



void setup()
{
  cam_servo.attach(cam_servo_pin);
  cur_angle=90;
  cam_servo.write(cur_angle);
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  pinMode(L_MOTOR_PWM, OUTPUT);
  pinMode(L_MOTOR_DIR1, OUTPUT);
  pinMode(L_MOTOR_DIR2, OUTPUT);
  
  pinMode(R_MOTOR_PWM, OUTPUT);
  pinMode(R_MOTOR_DIR1, OUTPUT);
  pinMode(R_MOTOR_DIR2, OUTPUT);
  
  // Init motors to stop
  digitalWrite(L_MOTOR_DIR1, LOW);
  digitalWrite(L_MOTOR_DIR2, LOW);
  digitalWrite(R_MOTOR_DIR1, LOW);
  digitalWrite(R_MOTOR_DIR2, LOW);
  analogWrite(L_MOTOR_PWM, 0);
  analogWrite(R_MOTOR_PWM, 0);

  //Encoder setup
  pinMode(r_encoder_pinA, INPUT);
  pinMode(r_encoder_pinB, INPUT);
  pinMode(l_encoder_pinA, INPUT);
  pinMode(l_encoder_pinB, INPUT);


  //Initialize Value
  pulse_l = 0;
  pulse_r = 0;
  timeold = 0;

  attachInterrupt(digitalPinToInterrupt(2), counter_r,CHANGE);
 
  attachPCINT(digitalPinToPCINT(l_encoder_pinA), counter_l, CHANGE);

  
  
  //ROS setup
  nh.initNode();
  Serial.begin(57600);
  nh.subscribe(sub_right);
  nh.subscribe(sub_left);
  nh.subscribe(sub_servo);

  //nh.advertise(rpm_left_pub);
  //nh.advertise(rpm_right_pub);
  nh.advertise(rticks_pub);
  nh.advertise(lticks_pub);
  delay(20);
}

void loop()
{
  if (millis() - timeold >= 100){  /*Uptade every one second, this will be equal to reading frecuency (Hz).*/
 
   rticks_msg.data = pulse_r;
   lticks_msg.data = pulse_l;

   //Publish data
   
   rticks_pub.publish(&rticks_msg);
   lticks_pub.publish(&lticks_msg);

   //reset parameter
   timeold = millis();
   //pulse_r = 0;
   //pulse_l = 0;
   
   //Write it out to serial port
   //Serial.print("RPM = ");
   //Serial.println(rpm_r,DEC);
   
   }

  
   //nh.loginfo("Log Me");
   nh.spinOnce();
   
}