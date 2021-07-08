#include "Arduino.h"
#include <SPI.h>
#include <RF24.h>

//RF24
#define PIN_CE 4
#define PIN_CSN 5

//DC motor controller
#define PIN_PWM_A 10
#define PIN_AIN1 7
#define PIN_AIN2 6


#define PIN_PWM_B 9
#define PIN_BIN1 8
#define PIN_BIN2 3


#define DEAD_ZONE 10
#define MAX_SPEED_PWM 255
#define MAX_STEERING_PWM 200
#define FWD 1
#define BWD 2
#define LEFT 1
#define RIGHT 2

struct radioDataReceive {
  uint8_t analog_left_X,
       analog_left_Y,
       analog_right_X,
       analog_right_Y,
       reserved0,
       reserved1,
       reserved2,
       potentiometer,
       control_mode,
       reserved3,
       bit_array,
       message_no;
};

struct machineState {
    float velocity_limit;
    uint8_t empty_receive_data;
    uint8_t empty_messages;
    uint8_t radio_print_counter;
    uint8_t speed;
    uint8_t speed_direction;
    uint8_t steering;
    uint8_t steering_direction;
};

class Motor
{
public:
	Motor(uint8_t pA, uint8_t pB, uint8_t pPWM);
	void forward(uint8_t speed, bool interlock);
	void backward(uint8_t speed, bool interlock);
	void stop();

private:
	int pinA,
		pinB,
		pinPWM;
};

Motor::Motor(uint8_t pA, uint8_t pB, uint8_t pPWM)
{
	Motor::pinA = pA;
	Motor::pinB = pB;
	Motor::pinPWM = pPWM;

	pinMode(pinA, OUTPUT);
	pinMode(pinB, OUTPUT);
	pinMode(pinPWM, OUTPUT);
}

void Motor::forward(uint8_t speed, bool interlock)
{
	digitalWrite(pinA, 0);
	digitalWrite(pinB, 1);
	if (interlock)
		analogWrite(pinPWM, speed);
	else
		analogWrite(pinPWM, 0);
}
void Motor::backward(uint8_t speed, bool interlock)
{
	digitalWrite(pinA, 1);
	digitalWrite(pinB, 0);
	if (interlock)
		analogWrite(pinPWM, speed);
	else
		analogWrite(pinPWM, 0);
}
void Motor::stop()
{
	digitalWrite(pinA, 0);
	digitalWrite(pinB, 0);
	analogWrite(pinPWM, 0);
}

RF24 radio(PIN_CE, PIN_CSN);
const byte rxAddr[6] = { '1','N','o','d','e','1' };
const byte txAddr[6] = { '1','N','o','d','e','2' };
radioDataReceive message_receive;
machineState machine_state;

Motor SpeedMotor(PIN_AIN1, PIN_AIN2, PIN_PWM_A);
Motor SteeringMotor(PIN_BIN1, PIN_BIN2, PIN_PWM_B);

unsigned long long now, last_radio_receive;

void radioConfig()
{
	radio.begin();
	radio.setDataRate(RF24_1MBPS);
	radio.setChannel(1);//100	
	// ------------   JAK SI� ROZJEBIE TO ZMIEN KANA� -----
	radio.openReadingPipe(0, rxAddr);
	radio.openWritingPipe(txAddr);
	radio.startListening();
}

void readRadio()
{
	if (radio.available())
    last_radio_receive = now;
	{	
        radioDataReceive last_message;
        if (machine_state.empty_receive_data == 0)		
            last_message = message_receive;
		radio.read(&message_receive, sizeof(message_receive));
		if (map(message_receive.potentiometer, 0, 255, 0, 100) > 20)
			machine_state.velocity_limit = float(map(message_receive.potentiometer, 0, 255, 0, 255));
		else
			machine_state.velocity_limit = 0.0;

        if ((message_receive.analog_left_Y == 0 && message_receive.analog_left_X == 0 &&
            message_receive.analog_right_X == 0 && message_receive.analog_right_Y == 0) || 
            (message_receive.analog_left_Y == 255 && message_receive.analog_left_X == 255 &&
            message_receive.analog_right_X == 255 && message_receive.analog_right_Y == 255))
        {
            machine_state.empty_receive_data = 1;
            machine_state.empty_messages++;
            if (machine_state.empty_messages < 10)
            {
                message_receive = last_message;
            }
        }
        else
        {
            machine_state.empty_receive_data = 0;
            machine_state.empty_messages = 0;
        }
        controlMotors();
        //delay(10);
	}

}

void printSpeed()
{
    if (machine_state.radio_print_counter < 10)
        Serial.print("  ");

    if (machine_state.radio_print_counter >= 10 && machine_state.radio_print_counter < 100)
        Serial.print(" ");

    Serial.print(machine_state.radio_print_counter);
    Serial.print(". ");
    Serial.print(machine_state.speed);
    Serial.print(" ");
    Serial.print(machine_state.speed_direction);
    Serial.print(" ");
    Serial.print(machine_state.steering);
    Serial.print(" ");
    Serial.print(machine_state.steering_direction);
    Serial.print("\t");
    Serial.print(message_receive.analog_left_Y);
    Serial.print(" ");
    Serial.print(message_receive.analog_right_X);
    Serial.print(" ");
    Serial.print(message_receive.message_no);
    Serial.print(" ");
    Serial.print(machine_state.empty_receive_data);
    Serial.print(" ");
    Serial.print(machine_state.empty_messages);
    Serial.print("\n");
    machine_state.radio_print_counter++;  
}

void controlMotors()
{
    // SPEED
    float speed_before_scaling;

    if (message_receive.analog_left_Y > 127 + DEAD_ZONE)
        {
            speed_before_scaling = map(message_receive.analog_left_Y, 127 + DEAD_ZONE , 255, 0, MAX_SPEED_PWM);
            // machine_state.speed = map(message_receive.analog_left_Y, 127 + DEAD_ZONE , 255, 0, MAX_SPEED_PWM);
            speed_before_scaling = speed_before_scaling *( (float)message_receive.potentiometer - 10) / 245.0;
            machine_state.speed = (uint8_t) speed_before_scaling;
            machine_state.speed_direction = FWD;
        }
    if (message_receive.analog_left_Y < 127 - DEAD_ZONE)
        {
            // machine_state.speed = map(message_receive.analog_left_Y, 0, 127 - DEAD_ZONE , MAX_SPEED_PWM, 0);
            speed_before_scaling = map(message_receive.analog_left_Y, 0, 127 - DEAD_ZONE , MAX_SPEED_PWM, 0);
            speed_before_scaling = speed_before_scaling *( (float)message_receive.potentiometer - 10) / 245.0;
            machine_state.speed = (uint8_t) speed_before_scaling;
            machine_state.speed_direction = BWD;
        }

    if (message_receive.analog_left_Y > 127 - DEAD_ZONE && message_receive.analog_left_Y < 127 + DEAD_ZONE || message_receive.potentiometer < 10)
        {
            machine_state.speed = 0;
            machine_state.speed_direction = FWD;
        }

    // STEERING

    if (message_receive.analog_right_X > 127 + DEAD_ZONE)
        {
            machine_state.steering = map(message_receive.analog_right_X, 127 + DEAD_ZONE , 255, 0, MAX_STEERING_PWM);
            machine_state.steering_direction = LEFT;
        }
    if (message_receive.analog_right_X < 127 - DEAD_ZONE)
        {
            machine_state.steering = map(message_receive.analog_right_X, 0, 127 - DEAD_ZONE , MAX_STEERING_PWM, 0);
            machine_state.steering_direction = RIGHT;
        }

    if (message_receive.analog_right_X > 127 - DEAD_ZONE && message_receive.analog_right_X < 127 + DEAD_ZONE)
        {
            machine_state.steering = 0;
            machine_state.steering_direction = LEFT;
        }    

    printSpeed();
}

void driveOutputs()
{
    if (machine_state.speed_direction == FWD)
    {
        SpeedMotor.forward(machine_state.speed, machine_state.speed > 0);
    }
    else if (machine_state.speed_direction == BWD)
    {
        SpeedMotor.backward(machine_state.speed, machine_state.speed > 0);
    }

    if (machine_state.steering_direction == LEFT)
    {
        SteeringMotor.forward(machine_state.steering, machine_state.steering > 0);
    }
    else if (machine_state.steering_direction == RIGHT)
    {
        SteeringMotor.backward(machine_state.steering, machine_state.steering > 0);
    }
}

void setup()
{
    radioConfig();
    Serial.begin(9600);
    Serial.println("RC car begin!");
}

void loop()
{
    now = millis();
    readRadio(); 
    if(now - last_radio_receive < 1000)
        driveOutputs();
    else
    {
        SteeringMotor.stop();
        SpeedMotor.stop();
        Serial.println("Radio failure!");
        radioConfig();
        delay(20);
        radio.flush_rx()
    }
    delay(1);
}