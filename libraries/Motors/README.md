MPORT# = PORT# (MPORT1 = PORT1, etc)      
Motors.h:  
Motor::Motor(int port, bool attachEnc = true)  
Motor::run(int speed)  
Motor::stop()  
Motor::getTicks()  
Motor::resetTicks()  

#define MOTORSOFF //to disable motors (place before inclusion in your file)
