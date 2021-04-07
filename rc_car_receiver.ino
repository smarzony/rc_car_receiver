#include <SPI.h>
#include <RF24.h>

#define PIN_PWM_2_CONTROL_2 3
#define PIN_CE 4
#define PIN_CSN 5
#define PIN_PWM_1_CONTROL_1 6
#define PIN_PWM_1_CONTROL_2 7
#define PIN_PWM_2_CONTROL_1 8
#define PIN_PWM_1 9
#define PIN_PWM_2 10

#define DEAD_ZONE 20
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

Motor SpeedMotor(PIN_PWM_1_CONTROL_1, PIN_PWM_1_CONTROL_2, PIN_PWM_1);
Motor SteeringMotor(PIN_PWM_2_CONTROL_1, PIN_PWM_2_CONTROL_2, PIN_PWM_2);

unsigned long long now;

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
	{				
		radio.read(&message_receive, sizeof(message_receive));
		if (map(message_receive.potentiometer, 0, 255, 0, 100) > 20)
			machine_state.velocity_limit = float(map(message_receive.potentiometer, 0, 255, 0, 255));
		else
			machine_state.velocity_limit = 0.0;
        
        // printRadio();
        controlMotors();
        delay(10);
	}
	if (message_receive.analog_left_Y == 0 && message_receive.analog_left_X &&
		message_receive.analog_right_X == 0 & message_receive.analog_right_Y == 0)
		machine_state.empty_receive_data = 1;
	else
		machine_state.empty_receive_data = 0;
}

void printRadio()
{
    Serial.print(machine_state.radio_print_counter);
    Serial.print(". ");
    Serial.print(message_receive.analog_left_Y);
    Serial.print(" ");
    Serial.print(message_receive.analog_left_X);
    Serial.print(" ");
    Serial.print(message_receive.analog_right_Y);
    Serial.print(" ");
    Serial.print(message_receive.analog_right_X);
    Serial.print("\n");  
    machine_state.radio_print_counter++;  
}

void printSpeed()
{
    Serial.print(machine_state.radio_print_counter);
    Serial.print(". ");
    Serial.print(machine_state.speed);
    Serial.print(" ");
    Serial.print(machine_state.speed_direction);
    Serial.print(" ");
    Serial.print(machine_state.steering);
    Serial.print(" ");
    Serial.print(machine_state.steering_direction);
    Serial.print("\n");
    machine_state.radio_print_counter++;  
}

void controlMotors()
{
    // SPEED

    if (message_receive.analog_left_Y > 127 + DEAD_ZONE)
        {
            machine_state.speed = map(message_receive.analog_left_Y, 127 + DEAD_ZONE , 255, 0, 255);
            machine_state.speed_direction = FWD;
        }
    if (message_receive.analog_left_Y < 127 - DEAD_ZONE)
        {
            machine_state.speed = map(message_receive.analog_left_Y, 0, 127 - DEAD_ZONE , 255, 0);
            machine_state.speed_direction = BWD;
        }

    if (message_receive.analog_left_Y > 127 - DEAD_ZONE && message_receive.analog_left_Y < 127 + DEAD_ZONE)
        {
            machine_state.speed = 0;
            machine_state.speed_direction = FWD;
        }

    // STEERING

    if (message_receive.analog_right_X > 127 + DEAD_ZONE)
        {
            machine_state.steering = map(message_receive.analog_right_X, 127 + DEAD_ZONE , 255, 0, 255);
            machine_state.steering_direction = RIGHT;
        }
    if (message_receive.analog_right_X < 127 - DEAD_ZONE)
        {
            machine_state.steering = map(message_receive.analog_right_X, 0, 127 - DEAD_ZONE , 255, 0);
            machine_state.steering_direction = LEFT;
        }

    if (message_receive.analog_right_X > 127 - DEAD_ZONE && message_receive.analog_right_X < 127 + DEAD_ZONE)
        {
            machine_state.steering = 0;
            machine_state.steering_direction = LEFT;
        }

    printSpeed();
}

void setup()
{
    radioConfig();
    Serial.begin(9600);
    Serial.println("RC car begin!");
}

void loop()
{
    readRadio(); 
}