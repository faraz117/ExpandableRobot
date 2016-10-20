
#include"Arduino.h"
namespace Robot_Ex
{
    class MotorDriver
    {
    public:
        /**
         * @brief Change the speed of the motor.
         * @param speed The new speed of the motor.
         *  Valid values are between -255 and 255. 
         *  Use positive values to run the motor forward, 
         *  negative values to run it backward and zero to stop the motor.
         */
        virtual void _setSpeed(int _speed) = 0;
        virtual void _move(int in1 , int in2 , int en , boolean state , int _speed);  
        /**
         * @brief Return the current speed of the motor.
         * @return The current speed of the motor with range -255 to 255.
         */
        virtual int getSpeed() const = 0;            
    };
};
