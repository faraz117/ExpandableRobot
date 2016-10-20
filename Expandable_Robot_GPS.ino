#include"L298_motor_driver.h"
#include"bluestick_remote_control.h"
#include <SoftwareSerial.h>
SoftwareSerial GPSSerial(14, 15); // RX, TX
#include"I2C_device.h"
#define r_bridge_in1 2
#define r_bridge_in2 3
#define r_bridge_en  9
#define l_bridge_in1 A3
#define l_bridge_in2 A2
#define l_bridge_en  10
#include "gps_robot.h"
#define TOO_CLOSE 17                    /**< distance to obstacle in centimeters */
#define MAX_DISTANCE (TOO_CLOSE * 20)   /**< maximum distance to track with sensor */
#define RANDOM_ANALOG_PIN 5             /**< unused analog pin to use as random seed */
#include <NewPing.h>
#include <Wire.h>
#include "newping_distance_sensor.h"
#define DISTANCE_SENSOR_INIT 4,5,MAX_DISTANCE
#define DISTANCE_SENSOR_INIT1 46,47,MAX_DISTANCE
#define DISTANCE_SENSOR_INIT2 11,12,MAX_DISTANCE
#define DISTANCE_SENSOR_INIT3 41,40,MAX_DISTANCE
#define REMOTE_CONTROL_INIT
#include "moving_average.h"
int alternate_speed;
float heading_req=180.00;
float prev_heading_req=0;
float heading_now;
int heading_error;
String command;
double lon=25.3935;
double lat=68.3334;
double dest_lon=26.0000;
double dest_lat=67.0000;
long current=0,previous=0;
namespace Robot_Ex
{
    class Robot
    {
    public:
        /*
         * @brief Class constructor.
         */
        Robot()
            : leftMotor(l_bridge_in1,l_bridge_in2,l_bridge_en), rightMotor(r_bridge_in1,r_bridge_in2,r_bridge_en),
         distanceSensor_front(DISTANCE_SENSOR_INIT),distanceSensor_front2(DISTANCE_SENSOR_INIT1),distanceSensor_down(DISTANCE_SENSOR_INIT2),distanceSensor_back(DISTANCE_SENSOR_INIT3),distanceAverage(TOO_CLOSE*10),distanceAverage2(TOO_CLOSE*10),groundHeight(TOO_CLOSE*10),distanceAverageback(TOO_CLOSE*10)
       ,remoteControl(REMOTE_CONTROL_INIT)  
        {
            initialize();
        }

        /*
         * @brief Initialize the robot state.
         */
        void initialize()
        {
           randomSeed(analogRead(RANDOM_ANALOG_PIN));
           remote();
        }

        /*
         * @brief Update the state of the robot based on input from sensor and remote control.
         *  Must be called repeatedly while the robot is in operation.
         */
        void run()
        {
            unsigned long currentTime = millis();
            int back = distanceAverageback.add(distanceSensor_back.getDistance());
            int ground= groundHeight.add(distanceSensor_down.getDistance());
            if( ground > 2 )
            {
              alternate_speed=100;
            }
            
            {
              alternate_speed=255;
            }
            
            int distance = distanceAverage.add(distanceSensor_front.getDistance());
            int distance2 = distanceAverage2.add(distanceSensor_front2.getDistance());
            RemoteControlDriver::command_t remoteCmd;
            bool haveRemoteCmd = remoteControl.getRemoteCommand(remoteCmd);
            float distance_remaining = calc_distance(lat,lon,dest_lat,dest_lon);
            if(distance_remaining < 4)
            {
              alternate_speed = 0;
              Serial.println("Waypoint Reached");
            }
            heading_req = courseToWaypoint(lat,lon,dest_lat,dest_lon);
            if(heading_req != prev_heading_req)
            {
            Serial.println(heading_req);
            prev_heading_req=heading_req;
            }
            heading_now = fetch_dc_values();
            heading_error = calcDesiredTurn(heading_req,heading_now);
            /*Serial3.print("Navigating to: ");
            Serial3.print(dest_lon);
            Serial3.print("  ");
            Serial3.print(dest_lat);
            Serial3.print("  ");
            Serial3.println(heading_req);*/
            Serial.print("Dwn: ");
            Serial.print(ground);
            Serial.print("  ");
            Serial.print("Lft: ");
            Serial.print(distance);
            Serial.print("  ");
            Serial.print("Rgt: ");
            Serial.print(distance2);
            Serial.print("   ");
            Serial.print("Ctr: ");
            Serial.print(back);
            Serial.print("    ");
            Serial.println(heading_error);
            
      if (remoteControlled()) {
                if (haveRemoteCmd) {
                    switch (remoteCmd.key) {
                    case RemoteControlDriver::command_t::keyF1:
                        // start "roomba" mode
                        _move(alternate_speed);
                        break;
                    case RemoteControlDriver::command_t::keyNone:
                        // this is a directional command
                        leftMotor._setSpeed(remoteCmd.left);
                        rightMotor._setSpeed(remoteCmd.right);
                        break;
                    default:
                        break;
                    }
                }
            }
            else {
                // "roomba" mode
                if (haveRemoteCmd && remoteCmd.key == RemoteControlDriver::command_t::keyF1) {
                    // switch back to remote mode
                    remote();
                }
                else {
                    if (moving()) {
                        if (obstacleAhead(distance,distance2 , back))
                            turn(currentTime,alternate_speed);
                        if (noFloor(ground))
                            {
                                reverse(currentTime);
                            }
                        if((heading_error < -10) || (heading_error > 10))
                           {
                             _adjust();
                           }   
                            //Serial.println("Moving");
                    }
                    
                    else if (turning()) {
                        if (doneTurning(currentTime, distance))
                            _move2(currentTime);
                  //Serial.println("Turning");  
                  }
                     else if (reverse())
                    {
                        if(doneReverse(currentTime,ground))
                          turn(currentTime,alternate_speed);
                    //Serial.println("Reversing");      
                    }
                    
                    else if (_move2())
                    {
                        if(doneMove2(currentTime))
                          _move(alternate_speed);
                        if (obstacleAhead(distance,distance2 , back))
                          turn(currentTime,alternate_speed);
                        if (noFloor(ground))
                            {
                                reverse(currentTime);
                            }     
                    }
                    else if (adjusting())
                    {
                      
                      if(doneAdjusting())
                      {
                        _move(alternate_speed);
                      }
                     //Serial.println(heading_now);
                    }
                }
            }
            
        }

    protected:
        void remote()
        {
            leftMotor._setSpeed(0);
            rightMotor._setSpeed(0);
            state = stateRemote;
        }
        void _move(int alternate_speed)
        {
            rightMotor._setSpeed(alternate_speed);
            leftMotor._setSpeed(alternate_speed);
            state = stateMoving;
        }

        
        void _stop()
        {
            leftMotor._setSpeed(0);
            rightMotor._setSpeed(0);
            state = stateStopped;
        }
        
        bool obstacleAhead(unsigned int distance , unsigned int distance2 , unsigned int distance3)
        {
            return (distance3 <= TOO_CLOSE || distance2 <=TOO_CLOSE || distance <= TOO_CLOSE);
        }
        
        bool obstacleFarAhead(unsigned int distance , unsigned int distance2, unsigned int distance3)
        {
            return ( (distance3 <=TOO_CLOSE*2 || distance <= TOO_CLOSE *2 || distance2 <=TOO_CLOSE*2) && (distance3 >=TOO_CLOSE ||distance >= TOO_CLOSE  || distance2 >=TOO_CLOSE));
        }
        
        bool noFloor (unsigned int distance )
        {
          return (distance >= 6);
        }
        
        bool turn(unsigned long currentTime, int alternate_speed)
        {
            if (random(2) == 0) {
                leftMotor._setSpeed(-1 * alternate_speed);
                rightMotor._setSpeed(alternate_speed);
            }
            else {
                leftMotor._setSpeed(alternate_speed);
                rightMotor._setSpeed(-1*alternate_speed);
            }
            state = stateTurning;
            endStateTime = currentTime + random(500, 750);
        }
        
        bool reverse(unsigned long currentTime)
        {
            rightMotor._setSpeed(-255);
            leftMotor._setSpeed(-255);
            state = stateReverse;
            endStateTime = currentTime + random(500, 800);
        }
        bool _move2(unsigned long currentTime)
        {
            rightMotor._setSpeed(255);
            leftMotor._setSpeed(255);
            state = stateMove2;
            endStateTime = currentTime + random(500, 800);
        }
        bool _adjust()
        {
          rightMotor._setSpeed(0);
          leftMotor._setSpeed(0);
            state=stateAdjust;
        }
        
        bool doneTurning(unsigned long currentTime, unsigned int distance)
        {
            if (currentTime >= endStateTime)
                return (distance > TOO_CLOSE);
            return false;
        }
         
         bool doneAdjusting()
        {
            heading_now= fetch_dc_values();
            heading_error = heading_req - heading_now;
            int pwm=heading_error*15;
            pwm=constrain(pwm,-255,255);
            rightMotor._setSpeed(pwm);
            leftMotor._setSpeed(-pwm);
            
            if((heading_error > -10) && (heading_error < 10))
                return true;
            return false;
            
        }
        
       bool doneReverse(unsigned long currentTime, unsigned int distance)
        {
            if (currentTime >= endStateTime)
                return (distance < 6);
            return false;
        }     
       bool doneMove2(unsigned long currentTime)
        {
            if (currentTime >= endStateTime)
                return true;
            return false;
        } 
        bool _move2() { return (state==stateMove2);}
        bool adjusting(){ return ( state == stateAdjust);}
        bool moving() { // Serial.println("MOVING");
      return (state == stateMoving); }
        bool turning() { //Serial.println("TURNING");
      return (state == stateTurning); }
        bool stopped() { return (state == stateStopped); }
        bool reverse() { //Serial.println("REVERSE");
      return (state == stateReverse );}
        bool remoteControlled() { //Serial.println("REMOTE");
      return (state == stateRemote); }

    private:
        Motor leftMotor;
        Motor rightMotor;
        DistanceSensor distanceSensor_front;
        DistanceSensor distanceSensor_front2;
        DistanceSensor distanceSensor_down;
        DistanceSensor distanceSensor_back;
        MovingAverage<unsigned int, 3> distanceAverage;
        MovingAverage<unsigned int, 3> distanceAverage2;
        MovingAverage<unsigned int, 3> distanceAverageback;
        MovingAverage<unsigned int, 3> groundHeight;
        enum state_t { stateStopped, stateMoving,stateMove2, stateAdjust, stateTurning , stateReverse ,stateRemote };
        state_t state;
        RemoteControl remoteControl;
        unsigned long endStateTime;
    };
};

Robot_Ex::Robot robot;

void setup()
{   init_magnetometer(); 
    robot.initialize();
    Serial.begin(57600);
    Serial2.begin(9600);
    Serial1.begin(56700);
    Serial1.println("Raspberry Pi Begin");
}

void loop()
{
  
   robot.run();
   feedgps();
   if(Serial.available())
   {
     char c = Serial.read();
     command=command+ c;
    if(c=='\n')
   {
     if(command.length() > 3)
     {
     Serial.println(command);
    // dest_lon=(command.substring(command.indexOf(',')+1)).toFloat();
     //dest_lat=(command.substring(0,command.indexOf(','))).toFloat();
     Serial.print(dest_lon,4);
     Serial.print("   ");
     Serial.println(dest_lat,4);
     command="";
     }
     else
     {
     command="";  
     }
   } 
   }
/*      if(Serial.available())
   {
     char c = Serial.read();
     command=command+ c;
    if(c=='\n')
   {
     Serial.println(command);
     dest_lon=(command.substring(command.indexOf(',')+1)).toFloat();
     dest_lat=(command.substring(0,command.indexOf(','))).toFloat();
     Serial.print(dest_lon,4);
     Serial.print("   ");
     Serial.println(dest_lat,4);
     command="";
   } 
   }*/
   ///get time here 
   /// Send data 
   /// check whether time for recieving data 
   /// recieve data and update variable 
   ///
   current=millis();
   if(current-previous > 1000)
   {
     float flat, flon;
     unsigned long age;
     gps.f_get_position(&flat, &flon, &age);
     if(flat !=10000.0000 && flon!=10000.0000)
    {
      lat=flat;
      lon=flon;
      Serial.print("Latitude: ");      
      Serial.print(flat, 4); 
      Serial.print(" Longitude: "); 
      Serial.println(flon, 4);
    };
   previous=current;
   }
}
