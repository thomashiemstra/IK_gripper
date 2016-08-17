#include <EEPROM.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define degtorad 0.0174533
#define radtodeg 57.2958
#define l1  11.8 //length arms in cm
#define l2  11.9
#define b  9 //height first rotational joint in cm
#define g 8 //gripper radius adjustment
#define gz 10.2 // gripper z direction adjustment
#define pi 3.14159265359

int g_s;
float i;
int pulse_length;
int Speed = 5 ;
int precession = 200;
float omega,a1,a2;
float x,y,z,r,z_p,x_a,y_a,z_a;
float x0,y0,z0,dx,dy,dz;
float angles[5];
float anglexy;

int servo[6][3]={  
  {135,325,530},
  {110,335,560},
  {110,340,555},
  {110,335,560},
  {150,370,600},
  {170,290,370}
};

void setup() 
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  z0=2;
  x0=0;
  y0=15;
  anglexy = atan2(y0,x0);
  r = sqrt(x0*x0+y0*y0);
  x_a = (r-g)*cos(anglexy);
  y_a = (r-g)*sin(anglexy);
  z_a = z0 + gz;
  inversekinematics(x_a,y_a,z_a);
  pwm.setPWM(5,0,170 );
  
}

void loop() 
{
  Serial.println("give x");
  while (Serial.available() == 0);
  x = Serial.parseFloat(); //read int or parseFloat for ..float...
  Serial.print("x = ");
  Serial.println(x);
  
  Serial.println("give y");
  while (Serial.available() == 0);
  y = Serial.parseFloat(); //read int or parseFloat for ..float...
  Serial.print("y = ");
  Serial.println(y);

  Serial.println("give gripper status, 1 for closed, 0 for open");
  while (Serial.available() == 0);
  g_s = Serial.parseFloat(); //read int or parseFloat for ..float...
  Serial.print("gripper status = ");
  Serial.println(g_s);
  
  dx= x - x0;
  dy= y - y0;

  for(i=0 ; i<precession ; i++)
  {
    z = z0 + ((i/precession) * 10);
    anglexy = atan2(y0,x0);
    r = sqrt(x0*x0+y0*y0);
    x_a = (r-g)*cos(anglexy);
    y_a = (r-g)*sin(anglexy);
    z_a = z + gz;
    inversekinematics(x_a,y_a,z_a);
    delay(5);
  }
  
  for(i=0 ; i<precession ; i++)
  {
    x = x0 + ((i/precession)*dx);
    y = y0 + ((i/precession)*dy); 
    anglexy = atan2(y,x);
    r = sqrt(x*x+y*y);
    x_a = (r-g)*cos(anglexy);
    y_a = (r-g)*sin(anglexy);
    z_a = z + gz;
    inversekinematics(x_a,y_a,z_a);
    delay(Speed*(r/20));
  }
  
  for(i=0 ; i<precession ; i++)
  {
    z = z0 + 10 - ((i/precession)*10);
    z_a = z + gz;
    inversekinematics(x_a,y_a,z_a);
    delay(5);
  }
  delay(500);
  pwm.setPWM(5,0,170 + g_s*200);
  x0 = x;
  y0 = y;
 
}


void inversekinematics (float x, float y, float z)
{
  z_p = z-b;
  r = sqrt(x*x+y*y); 
  angles[0] = radtodeg*atan2(y,x);
  a1 =  sqrt( 1 - sq( (r*r+z_p*z_p-l1*l1-l2*l2)/(2*l1*l2) ) );
  a2 = (r*r+z_p*z_p-l1*l1-l2*l2)/(2*l1*l2);
  float res = atan2(-a1,a2);
  angles[2] = -radtodeg*res;
  float k1 = l1+l2*cos(res);
  float k2 = l2*sin(res);
  angles[1] = radtodeg*(atan2(z_p,r) - atan2(k2,k1));
  servoangle(0,0,angles[0]);
  servoangle(1,1,angles[1]);
  servoangle(2,2,angles[1]);
  servoangle(3,3,angles[2]);
  angles[3] = 45 - angles[1] + angles[2];
  servoangle(4,4,angles[3]);
  
}

void servoangle(int servo_number, int channel, float angle)
{
    int n = servo_number; //n so that the formulas don't get cluttered with servo_angle everywhere
    if (angle >= 0 && angle <= 90)
    {
      pulse_length = int(float(servo[n][0]+angle*( (servo[n][1]-servo[n][0]) )/90.0));  
    }    
    else if(angle > 90 && angle <= 180)
    {
      pulse_length = int(float(  servo[n][1] + (angle-90.0) *( (servo[n][2]-servo[n][1]))/90.0 ) );     
    }
    else // if (angle <0 && angle>180) //redudant protection just in case
    {
      return;
    }
    pwm.setPWM(channel, 0, pulse_length);
}


