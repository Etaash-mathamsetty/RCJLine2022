utils::setMotors(Motor* motor1, Motor* motor2)  
utils::forward(int right_speed, int left_speed)  
utils::forward(int speed)  
utils::forwardTicks(int speed, int ticks)  
utils::stopMotors()  
utils::resetTicks()  
utils::logger::begin()  
utils::logger::println()  
utils::logger::println(auto thing)  
utils::logger::print(auto thing)  

in robot_code.ino  
#define log utils::logger  
so,  
log::println(auto thing) and etc, works too  
