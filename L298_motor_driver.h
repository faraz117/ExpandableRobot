
#include "motor_driver.h"
#include "Arduino.h"

namespace Robot_Ex
{
    class Motor : public MotorDriver
    {
    public:
        /*
         * @brief Class constructor.
         * @param pins to control the motors 
         */
        Motor(int mc1,int mc2,int mc3)
            : MotorDriver(), currentSpeed(0)
        {
          pinMode(mc1,OUTPUT);
          pinMode(mc2,OUTPUT);
          pinMode(mc3,OUTPUT);
          input1=mc1;
          input2=mc2;
          enable=mc3;
        }
	
        void _move(int in1 , int in2 , int en , boolean state , int _speed)
	{  
		  digitalWrite(in1,state);
		  digitalWrite(in2,!state);
		  analogWrite(en , _speed);
	}	
        void _setSpeed(int _speed )
        {
            currentSpeed = _speed;
            if (_speed >= 0) {
            _move(input1 , input2 , enable , HIGH ,  _speed);			  
				
        }
	else{

            _move(input1 , input2 , enable , LOW , (-1)*  _speed);
            }
        }

        int getSpeed() const
        {
            return currentSpeed;
        }

    private:
        
        int currentSpeed;
        int input1,input2,enable;
    };
};
