#include "remote_control.h"

namespace Robot_Ex
{
    class RemoteControl : public RemoteControlDriver
    {
    public:
        /**
          * @brief Class constructor.
          */
        RemoteControl() : RemoteControlDriver(), lastKey(command_t::keyNone) {}

        virtual bool getRemoteCommand(command_t& cmd)
        {
            cmd.stop();
            cmd.key = command_t::keyNone;
            
            if ( Serial2.available() <= 0 && Serial1.available() <= 0 )
            {
                return false; // no commands available
            }
            else
            {   
            char ch ;  
             if(Serial2.available () > 0)
            { 
                ch = Serial2.read();
            }
             if(Serial1.available () > 0)
            { 
                ch = Serial1.read();
            }


                          
                
                switch (ch) {
                    case '8': // up
                        cmd.goForward();
                        break;
                    case '2': // down
                        cmd.goBack();
                        break;
                    case '4': // left
                        cmd.turnLeft();
                        break;
                    case '6': // right
                        cmd.turnRight();
                        break;
                    case '5': // right
                        cmd.stop();
                        break;    
                    case 'A': // function key #1
                    case 'C':
                        cmd.key = command_t::keyF1;
                        break;
                    case 'B': // function key #2
                    case 'D':
                        cmd.key = command_t::keyF2;
                        break;
                    case 'E': // function key #3
                        cmd.key = command_t::keyF3;
                        break;
                    case 'F': // function key #4
                        cmd.key = command_t::keyF4;
                        break;
                    default:
                        break;
                }
          if (cmd.key != command_t::keyNone && cmd.key == lastKey) {
                // repeated key, ignore it
                return false; 
            }
            lastKey = cmd.key;
            return true;
        }
       }
    
    private:
        command_t::key_t lastKey;
    };
};
