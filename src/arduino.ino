#include <ros.h>
//#include <std_msgs/Float32.h>                             
#include <std_msgs/Float32MultiArray.h>

#include <MultiArrayDimension.h>

#include <Encoder.h>

#define wheel_radius 0.11
#define wheel_sep 4.5

const int motor_en_left = 10;
const int motor_dir_left = 5;

const int motor_en_right = 6;
const int motor_dir_right = 4;

Encoder myEnc_L(2, 3);
Encoder myEnc_R(8,9);

ros::NodeHandle nh;

//std_msgs::Float32 pwm_left_msg;
std_msgs::Float32MultiArray encoder_msg;
ros::Publisher encoder_publisher("encoder",&encoder_msg);
//std_msgs::Float32MultiArray encoder_msg;
//ros::Publisher pwm_publisher_left("encoder_left", &pwm_left_msg);



void velocity_callback( const std_msgs::Float32MultiArray &msg)
{
data_rawdata_raw
// float dstride=msg->layout.dim;
// int size=msg->layout.dim.size;
  


  // --------- ENCODER PUBLISHER --------------


  //encoder_msg.layout.dim.push_back(std_msgs::MultiArrayDimension)
  float newPosition_L = myEnc_L.read();  
  encoder_msg.data[0]=newPosition_L ;
  float newPosition_R = myEnc_R.read();  
  encoder_msg.data[1]= newPosition_R;
  //pwm_publisher_left.publish(&pwm_left_msg);
  encoder_publisher.publish(&encoder_msg);

 
  // -------- VELOCITY SUBSCRIBER --------------
  float w_l = msg->data[0];
  float w_r = msg->data[1];
  //w_l_mapped = map(w_l,from_low,from_high,to_low,to_high)
  //w_l_mapped = map(w_l,from_low,from_high,to_low,to_high)
  int vel_map_l = map(w_l,0,125,0,255);
  int vel_map_r = map(w_r,0,125,0,255);
 
 
  if (vel_map_l >=0)
  {
    analogWrite(motor_en_left,vel_map_l);
    digitalWrite(motor_dir_left,LOW);
  }
  else
  {
    analogWrite(motor_en_left,vel_map_l);
    digitalWrite(motor_dir_left,HIGH);
  }

  if (vel_map_r >=0)
  {
    analogWrite(motor_en_right,vel_map_r);
    digitalWrite(motor_dir_right,LOW);
  }
  else
  {
    analogWrite(motor_en_right,vel_map_r);
    digitalWrite(motor_dir_right,HIGH);
  }

}

ros::Subscriber<std_msgs::Float32MultiArray> sub("/VelPub",2 ,velocity_callback);


void setup() {
  // put your setup code here, to run once:
 
  pinMode(motor_en_left, LOW);

  nh.initNode();
  encoder_msg.layout.dim = (std_msgs::MultiArrayDimension *)
  encoder_msg.layout.dim[0].label="height";
  encoder_msg.layout.dim[0].size=2;
  encoder_msg.layout.dim[0].stride=1*2;
  encoder_msg.layout.data_offset=0;
  encoder_msg.data=(int *)malloc(sizeof(float)*4);
  encoder_msg.layout.dim_length = 1;
  
  encoder_msg.data_length=4;
  nh.advertise(encoder_publisher);
  nh.subscribe(sub);
 
}

void loop() {
  // put your main code here, to run repeatedly:
 
  nh.spinOnce();

}

