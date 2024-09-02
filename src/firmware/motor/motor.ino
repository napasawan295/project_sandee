#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <Encoder.h>
#include <PID_v1.h>


Encoder myEncL(2, 4);
Encoder myEncR(18, 19);

long oldPositionL = 0;
long oldPositionR = 0;
ros::NodeHandle nh;


geometry_msgs::Twist cmd_vel;
std_msgs::Float32 encL_msg;
std_msgs::Float32 encR_msg;


ros::Publisher EncL("Enc_L", &encL_msg);
ros::Publisher EncR("Enc_R", &encR_msg);

long newPositionL;
long newPositionR;

// กำหนดค่าตัวแปรสำหรับ PID
double setpointL, inputL, outputL;
double setpointR, inputR, outputR;

// กำหนดค่าคงที่สำหรับ PID (Kp, Ki, Kd)
double Kp = 2.0, Ki = 0.1, Kd = 0.01;
PID pidL(&inputL, &outputL, &setpointL, Kp, Ki, Kd, DIRECT);
PID pidR(&inputR, &outputR, &setpointR, Kp, Ki, Kd, DIRECT);

// ฟังก์ชัน callback สำหรับรับคำสั่งจาก ROS
void roverCallBack(const geometry_msgs::Twist& cmd_vel)
{
  double x = cmd_vel.linear.x;
  double z = cmd_vel.angular.z;

  double outputL = x + (z / 2);
  double outputR = x - (z / 2);
  
  // ควบคุมมอเตอร์ซ้าย
  if (outputL > 0.0) {
    analogWrite(6, max(min(outputL * 200, 90), 45));
    digitalWrite(7, LOW);
  } else if (outputL < 0.0) {
    analogWrite(6, max(min(abs(outputL) * 200, 90), 45));
    digitalWrite(7, HIGH);
  } else {
    analogWrite(6, 0);
    digitalWrite(7, LOW);
  }

  // ควบคุมมอเตอร์ขวา
  if (outputR > 0.0) {
    analogWrite(44, max(min(outputR * 200, 90), 45));
    digitalWrite(45, LOW);
  } else if (outputR < 0.0) {
    analogWrite(44, max(min(abs(outputR) * 200, 90), 45));
    digitalWrite(45, HIGH);
  } else {
    analogWrite(44, 0);
    digitalWrite(45, LOW);
  }
}

ros::Subscriber<geometry_msgs::Twist> Motor("/cmd_vel", roverCallBack);

void setup()
{
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);

  nh.initNode();
  nh.subscribe(Motor);
  nh.advertise(EncL);
  nh.advertise(EncR);

  // ตั้งค่าโหมดสำหรับ PID
  pidL.SetMode(AUTOMATIC);
  pidR.SetMode(AUTOMATIC);
}

void loop()
{
  // อ่านค่า Encoder ใหม่
  newPositionL = myEncL.read();
  newPositionR = myEncR.read() * -1;

  // คำนวณความเร็วที่แท้จริง (ค่า input สำหรับ PID)
  inputL = newPositionL - oldPositionL;
  inputR = newPositionR - oldPositionR;

  // คำนวณ PID
  pidL.Compute();
  pidR.Compute();

  // ตรวจสอบตำแหน่งใหม่และปรับปรุงค่า Encoder
  if (newPositionL != oldPositionL) {
    oldPositionL = newPositionL;
    encL_msg.data = newPositionL;
    EncL.publish(&encL_msg);
  }
  if (newPositionR != oldPositionR) {
    oldPositionR = newPositionR;
    encR_msg.data = newPositionR;
    EncR.publish(&encR_msg);
  }

  // ตรวจสอบค่าสูงสุด/ต่ำสุดของ Encoder และรีเซ็ตถ้าจำเป็น
  if (abs(newPositionL) > 100000000 || abs(newPositionR) > 100000000) {
    myEncL.write(0);
    myEncR.write(0);
  }

  nh.spinOnce();
  delay(10);
}
