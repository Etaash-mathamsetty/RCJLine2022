#ifndef MOTOR_CODE
#define MOTOR_CODE

#define MPORT1 0
#define MPORT2 1
#define MPORT3 2
#define MPORT4 3


struct motor_pins {
    uint8_t h1;
    uint8_t h2;
    uint8_t pwm;
    uint8_t cha;
};


const motor_pins pi_motor_pins[4] = { {35,34,12,18},{36,37, 8,19},{42,43, 9,3},{A5,A4,5,2} };

class Motor {
public:
    Motor(int port, bool attachEnc = true) {
        this->port = port;
        if (attachEnc)
            attachEncoder();
        //  mticks = ticks[port];

            //The PWM frequency is 976 Hz
#if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU
        TCCR1A = _BV(WGM10);
        TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

        TCCR3A = _BV(WGM30);
        TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

        TCCR4B = _BV(CS42) | _BV(CS41) | _BV(CS40);
        TCCR4D = 0;

#elif defined(__AVR_ATmega328__) // else ATmega328

        TCCR1A = _BV(WGM10);
        TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

        TCCR2A = _BV(WGM21) | _BV(WGM20);
        TCCR2B = _BV(CS22);

#elif defined(__AVR_ATmega2560__) //else ATmega2560
        TCCR1A = _BV(WGM10);
        TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

        TCCR2A = _BV(WGM21) | _BV(WGM20);
        TCCR2B = _BV(CS22);
#endif
    }



    void run(int speed) {
#ifndef MOTORSOFF
        speed = speed > 255 ? 255 : speed;
        speed = speed < -255 ? -255 : speed;
        if (speed >= 0) {
            digitalWrite(pi_motor_pins[port].h2, LOW);
            digitalWrite(pi_motor_pins[port].h1, HIGH);
            analogWrite(pi_motor_pins[port].pwm, speed);
            dir[port] = true;

        }
        if (speed < 0) {
            digitalWrite(pi_motor_pins[port].h1, LOW);
            digitalWrite(pi_motor_pins[port].h2, HIGH);
            analogWrite(pi_motor_pins[port].pwm, -speed);
            dir[port] = false;
        }
#endif
    }

    void stop() {
        run(0);
    }

    void attachEncoder() {
        if (port == 0)
            attachInterrupt(digitalPinToInterrupt(pi_motor_pins[port].cha), interupt0, RISING);
        if (port == 1)
            attachInterrupt(digitalPinToInterrupt(pi_motor_pins[port].cha), interupt1, RISING);
        if (port == 2)
            attachInterrupt(digitalPinToInterrupt(pi_motor_pins[port].cha), interupt2, RISING);
        if (port == 3)
            attachInterrupt(digitalPinToInterrupt(pi_motor_pins[port].cha), interupt3, RISING);
    }

    static void interupt0() {
        if (dir[0])
            ticks[0]++;
        else
            ticks[0]--;
    }

    static void interupt1() {
        if (dir[1])
            ticks[1]++;
        else
            ticks[1]--;
    }

    static void interupt2() {
        if (dir[2])
            ticks[2]++;
        else
            ticks[2]--;
    }


    static void interupt3() {
        if (dir[3])
            ticks[3]++;
        else
            ticks[3]--;
    }

    int getTicks() {
        return ticks[port];
    }

    void resetTicks() {
        ticks[port] = 0;
    }
    //int& mticks;

    int port;
    static inline int ticks[4] = { 0 };
    static inline bool dir[4] = { true };
};
#endif MOTOR_CODE
