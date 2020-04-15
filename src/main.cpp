#include <Arduino.h>

#include <PID_v1.h>

#define encoderPinA 18
#define encoderPinB 19

int enablePin = 5; //h bridge enable pin 1 for left motor
int inputPin1 = 4; // h bridge input 1 for left mootor

double Kp = 0, Ki = 5, Kd = 0;
double position, speed, setPoint;
volatile double intPosition;

PID myPid(&position, &speed, &setPoint, Kp, Ki, Kd, AUTOMATIC);



void leftISR()
{
	if (digitalRead(encoderPinA) == LOW)
	{
		if (digitalRead(encoderPinB) == HIGH)
		{
			intPosition++;
			Serial.print("POSISTION: ");
			Serial.println(intPosition);
		}
	}

}

void setSpeed(float speed)
{
	analogWrite(enablePin, speed);
}

void setup()
{
	Serial.begin(115200);
	speed = 0;
	setPoint = 87;
	position =0;
	intPosition = 0;
	
	myPid.SetMode(AUTOMATIC);
	myPid.SetSampleTime(1);

	pinMode(enablePin, OUTPUT);
	pinMode(inputPin1, OUTPUT);
	digitalWrite(inputPin1, HIGH);

	attachInterrupt(digitalPinToInterrupt(19), leftISR, CHANGE);
}


void loop()
{

	position = intPosition;

	myPid.Compute();

	if (position < setPoint)
	{
		setSpeed(speed);
	}
	Serial.print("Speed: ");
	Serial.println(speed);

}