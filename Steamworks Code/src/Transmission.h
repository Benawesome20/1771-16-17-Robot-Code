/*
 * Transmission.h
 *
 *  Created on: January 21, 2017
 *      Author: Benjamin Ventimiglia
 */

#ifndef SRC_TRANSMISSION_H_
#define SRC_TRANSMISSION_H_

#include <WPILib.h>
#include <CANTalon.h>

class Transmission {
	CANTalon motor;
	CANTalon motor2;
	Encoder encoder;
	bool twice = 0;

public:
	/* For just a motor encoder pair, sets motor2 to a nonexistent port to ensure it is never used */
	Transmission(int motor_port, int encoder_channel1, int encoder_channel2):
		motor(motor_port),
		motor2(1771),
		encoder(encoder_channel1, encoder_channel2)
	{
	}

	/* For actual transmissions with two Talons */
	Transmission(int motor_port, int motor2_port, int encoder_channel1, int encoder_channel2):
		motor(motor_port),
		motor2(motor2_port),
		encoder(encoder_channel1, encoder_channel2)
	{
		twice = 1;
	}

	/* Sets the speed of the motor */
	void Set(double rate)
	{
		if (twice)
		{
			motor.Set(rate);
			motor2.Set(rate);
		}
		else
			motor.Set(rate);
	}

	void ConfigNeutralMode(CANTalon::NeutralMode mode)
	{
		motor.ConfigNeutralMode(mode);
	}

	double Get1OutputCurrent()
	{
		return motor.GetOutputCurrent();
	}

	double Get2OutputCurrent()
	{
		return motor2.GetOutputCurrent();
	}

	double GetAvgOutputCurrent()
	{
		return (motor.GetOutputCurrent() + motor2.GetOutputCurrent()) / 2;
	}

	/* Resets the encoder value */
	void Reset()
	{
		encoder.Reset();
	}

	/* Gets the distance driven by the motor */
	double GetDistance()
	{
		return encoder.GetDistance();
	}

	double GetRawEncoderValue()
	{
		return encoder.Get();
	}

	/* Gets the speed of the motor */
	double GetSpeed()
	{
		return encoder.GetRate();
	}

};

#endif /* SRC_TRANSMISSION_H_ */
