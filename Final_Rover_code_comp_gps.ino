#include <Wire.h>
#include "QMC5883L.h"
#include <TinyGPS++.h>
#define Task_t 10
#define gps_serial Serial3
#define bt_serial Serial2

TinyGPSPlus gps;
QMC5883L compass;

int gps_course, no_of_sats, bt_val, ac = 0, wp_count = 0, compass_dev = 5, pass = 0, dt = 0, increment = 0;
int compass_heading, desired_heading, heading_a, heading_b;
String str;
float lat_current = 0, lng_current = 0, scaled_factor = 1000000.0;
unsigned long currentMillis = 0, previousMillis = 0, t, dist_to_home;
const long interval = 200;
double home_lat_arr[10];
double home_lng_arr[10];

const int pwm_front_left = 3;
const int pwm_front_right = 4;
const int pwm_mid_left = 5;
const int pwm_mid_right = 6;
const int pwm_rear_left = 7;
const int pwm_rear_right = 8;

const int dir_front_left = 40;
const int dir_front_right = 41;
const int dir_mid_left = 42;
const int dir_mid_right = 43;
const int dir_rear_left = 44;
const int dir_rear_right = 45;

const int pwm_limit = 50;

void setup()
{
  Serial.begin(9600);
  gps_serial.begin(9600);
  bt_serial.begin(9600);
  //mot_serial.begin(9600);
  Wire.begin();
  pinMode(pwm_front_left, OUTPUT);
  pinMode(dir_front_left, OUTPUT);
  pinMode(pwm_front_right, OUTPUT);
  pinMode(dir_front_right, OUTPUT);
  pinMode(pwm_mid_left, OUTPUT);
  pinMode(dir_mid_left, OUTPUT);
  pinMode(pwm_mid_right, OUTPUT);
  pinMode(dir_mid_right, OUTPUT);
  pinMode(pwm_rear_left, OUTPUT);
  pinMode(dir_rear_left, OUTPUT);
  pinMode(pwm_rear_right, OUTPUT);
  pinMode(dir_rear_right, OUTPUT);   

  //Initializing the compass
  compass.init();
  compass.setOversampling(512);
  compass.setRange(8);
  compass.setSamplingRate(100);

  Serial.println("Calibrating the compass and Searching GPS satellites.....wait!");
  while(no_of_sats <= 4)
  {
    get_gps();
    no_of_sats = (int)(gps.satellites.value());
  }
  delay(5000);
  set_wp();
  wp_count = 0;
  ac = 0;
  delay(100);

  Serial.print(no_of_sats);
  Serial.print("  Satellites received ");
  delay(100);
}

void loop()
{
  bt();
  delay(200);
  get_gps();
  delay(200);
  get_compass();
  delay(200);
}

void bt()
{
  while(bt_serial.available() > 0)
  {
    str = bt_serial.readStringUntil('\n');
    bt_val = (str.toInt());

    switch(bt_val)
    {
      case 1:   forward();
                break;
      case 2:   backward();
                break;
      case 3:   left_turn();
                break;
      case 4:   right_turn();
                break;
      case 5:   stop_it();
                break;
      case 6:   set_wp();
                break;
      case 7:   go_to_wp();
                break;
      case 8:   //turn_180();
                break;
      case 9:   //compass_forward();
                break;
      case 10:  //set_heading();
                break;
      case 11:  gps_info();
                break;
      case 12:  //compass_right();
                break;
      case 13:  //compass_left();
                break;
      case 14:  //compass_calibrate();
                break;
      case 15:  //ping();
                break;
      case 16:  clr_wp();
                break;   
      case 17:  ac = 0;
                Serial.print(" WayPoints complete....!");
                break;      
    }
  }

  if(bt_serial.available() < 0)
  {
    Serial.println("Sorry...! No BT data received");
  }
}

void get_gps()
{
  while(gps_serial.available())
  {
    gps.encode(gps_serial.read());
  }
}

void get_compass()
{
  compass_heading = compass.readHeading();
  if(compass_heading >= 350 && compass_heading <= 360)
  {
    compass_heading = 90;
  }

  else if(compass_heading >= 0 && compass_heading <= 10)
  {
    compass_heading = 90;
  }

  else if(compass_heading >= 75 && compass_heading <= 105)
  {
    compass_heading = 0;
  }

  else if(compass_heading >= 165 && compass_heading <= 195)
  {
    compass_heading = 270;
  }

  else if(compass_heading >= 250 && compass_heading <= 290)
  {
    compass_heading = 180;
  }
  delay(200);
}

void set_wp()
{
  if(wp_count >= 0)
  {
    Serial.print("GPS Wp ");
    Serial.print(wp_count + 1);
    Serial.print(" Set ");
    get_gps();
    get_compass();

    home_lat_arr[ac] = lat_current;
    home_lng_arr[ac] = lng_current;

    for(int i = 0;i < wp_count;i++)
    {
      Serial.print("Waypoint ");
      Serial.print(i+1);
      Serial.println(" Set.");
      Serial.print("Latitude : ");
      Serial.print(home_lat_arr[i], 6);
      Serial.print("\tLongitude : ");
      Serial.print(home_lng_arr[i], 6);
      Serial.print("\n\n");
    }

    wp_count++;
    ac++;
  }

  else
  {
    Serial.print("All Wp are full...!");
    delay(500);
  }
}

void clr_wp()
{
  memset(home_lat_arr, 0, sizeof(home_lat_arr));
  memset(home_lng_arr, 0, sizeof(home_lng_arr));
  wp_count = 0;
  ac = 0;
  Serial.print("GPS Waypoints cleared..!");
  delay(500);
}

void go_to_wp()
{
  get_compass();
  while(true)
  {
    bt();
    if(bt_val == 5)
    {
      break;
    }
    get_gps();

    if(millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println("No GPS data....check wiring");
      delay(500);
    }

    dist_to_home = gps.distanceBetween(gps.location.lat(), gps.location.lng(), home_lat_arr[ac], home_lng_arr[ac]);
    gps_course = gps.courseTo(gps.location.lat(), gps.location.lng(), home_lat_arr[ac], home_lng_arr[ac]);
    float val = 0.000100 + home_lat_arr[0];
    float vol = home_lng_arr[0] - 0.000100;
    Serial.print("Distance to Home : ");
    Serial.println(dist_to_home);
    delay(200);  
    
    if((dist_to_home >= 0) && (dist_to_home < 5))
    {
      stop_it();
      Serial.print("Arrived at your destination...!");
      ac++;
      break;
      delay(1000);
    }

    if(abs(gps_course - compass_heading) <= 20)
    {
      forward();
      delay(500);
    }

    else
    {
      int x = (gps_course - 360);
      int y = (compass_heading - (x));
      int z = (y - 360);

      if((z >= 0) && (z <= 180))
      {
        left_turn();
        delay(500);
      }
      else
      {
        right_turn();
        delay(500);
      }
    }
  }
}

void gps_info()
{
  no_of_sats = (int)(gps.satellites.value());
  dist_to_home = gps.distanceBetween(gps.location.lat(), gps.location.lng(), home_lat_arr[ac], home_lng_arr[ac]);
  Serial.print("Latitude : ");
  Serial.print(gps.location.lat(), 6);
  Serial.print("Longitude : ");
  Serial.print(gps.location.lng(), 6);
  Serial.print(" ");
  Serial.print(no_of_sats);
  Serial.print("SATS");
  Serial.print(" ");
  Serial.print(dist_to_home);
  Serial.print("m");
  Serial.print("Distance from Home");
}

/*
void set_heading()
{
  for(int i = 0;i<=5;i++)
  {
    get_compass();
  }

  desired_heading = compass_heading;
  heading_a = compass_heading;
  heading_b = compass_heading + 180;

  if(heading_b >= 360)
  {
    heading_b = heading_b - 360;
  }

  Serial.print("Compass Heading Set : ");
  Serial.print(compass_heading);
  Serial.print(" Degrees ");
  Serial.print("Desired Heading");
  Serial.println(desired_heading);
  Serial.print("Compass Heading");
  Serial.println(compass_heading);
}

void compass_right()
{
  stop();
  get_compass();
  desired_heading = (desired_heading + 90);
  if(desired_heading >= 360)
  {
    desired_heading = desired_heading - 360;
  }

  while(abs(desired_heading - compass_heading) >= compass_dev)
  {
    get_compass();
    bt();
    if(bt_val == 5)
    {
      break;
    }
    if(desired_heading >= 360) 
    {
      desired_heading = desired_heading - 360;
    }

    int x = desired_heading - 359;
    int y = compass_heading - (x);
    int z = (y - 360);

    if((z <= 180) && (z >= 0))
    {
      left();
    }
    else
    {
      right();
    }

    stop();
  }
}

void turn_left()
{
  stop();
  get_compass();
  desired_heading = desired_heading - 90;
  if(desired_heading <= 0)
  {
    desired_heading = desired_heading + 360;
  }
  while(abs(desired_heading - compass_heading) >= compass_dev)
  {
    get_compass();
    bt();
    if(bt_val == 5);
    {
      break;
    }
    
    if(desired_heading >= 360)
    {
      desired_heading = desired_heading - 360;
    }

    int x = desired_heading - 359;
    int y = compass_heading - (x);
    int z = y - 360;

    if((z <= 180) && (z >= 0))
    {
      left();
    }
    else
    {
      right();
    }

    stop();
  }
}

void compass_forward()
{
  while(bt_val == 9)
  {
    get_compass();
    bt();
    if(bt_val == 5)
    {
      break;
    }

    if(abs(desired_heading - compass_heading) <= compass_dev)
    {
      forward();
    }
    else
    {
      int x = desired_heading - 359;
      int y = compass_heading - (x);
      int z = y - 360;
      
      if((z <= 180) && (z >= 0))
      {
        left();
      }
      else
      {
        right();
      }
    }
  }
}

void turn_180()
{
  if(pass == 0)
  {
    compass_right();
  }
  else
  {
    compass_left();
  }

  stop();

  if(pass == 0)
  {
    compass_right()
  }
}
*/

void increase_motor_pwm()
{
  for(int i = 0;i <= pwm_limit; i += 10)
  {
    analogWrite(pwm_front_left, i);
    analogWrite(pwm_front_right, i);
    analogWrite(pwm_mid_left, i);
    analogWrite(pwm_mid_right, i);
    analogWrite(pwm_rear_left, i);
    analogWrite(pwm_rear_right, i);
    delay(100);
  }
}

void decrease_motor_pwm()
{
  for(int i = pwm_limit; i >= 0; i -= 10)
  {
    analogWrite(pwm_front_left, i);
    analogWrite(pwm_front_right, i);
    analogWrite(pwm_mid_left, i);
    analogWrite(pwm_mid_right, i);
    analogWrite(pwm_rear_left, i);
    analogWrite(pwm_rear_right, i);
    delay(100);
  }
}

void forward()
{
  digitalWrite(dir_front_left, LOW);
  digitalWrite(dir_front_right, HIGH);
  digitalWrite(dir_mid_left, LOW);
  digitalWrite(dir_mid_right, HIGH);
  digitalWrite(dir_rear_left, LOW);
  digitalWrite(dir_rear_right, HIGH);
  delay(100);
  increase_motor_pwm();
}

void backward()
{
  digitalWrite(dir_front_left, HIGH);
  digitalWrite(dir_front_right, LOW);
  digitalWrite(dir_mid_left, HIGH);
  digitalWrite(dir_mid_right, LOW);
  digitalWrite(dir_rear_left, HIGH);
  digitalWrite(dir_rear_right, LOW);
  delay(100);
  increase_motor_pwm();
}


void left_turn()
{
  digitalWrite(dir_front_left, HIGH);
  digitalWrite(dir_front_right, HIGH);
  digitalWrite(dir_mid_left, HIGH);
  digitalWrite(dir_mid_right, HIGH);
  digitalWrite(dir_rear_left, HIGH);
  digitalWrite(dir_rear_right, HIGH);
  delay(100);
  increase_motor_pwm();
}

void right_turn()
{
  digitalWrite(dir_front_left, LOW);
  digitalWrite(dir_front_right, LOW);
  digitalWrite(dir_mid_left, LOW);
  digitalWrite(dir_mid_right, LOW);
  digitalWrite(dir_rear_left, LOW);
  digitalWrite(dir_rear_right, LOW);
  delay(100);
  increase_motor_pwm();
}

void stop_it()
{
  decrease_motor_pwm();
  delay(200);
}

