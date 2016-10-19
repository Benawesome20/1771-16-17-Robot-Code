#include "WPILib.h"
#include<thread>
#include <Math.h>

using namespace std;

#define DEBUG 								//Debug mode, shows stats as well as other functions for testing
//#define THROTTLE						//Sets rollers to be controlled by the a_stick throttle
//#define ARM_JOYSTICK_CONTROL

#define TRACKMULT .9 						//Multiplier for vision tracking speed

#define GEAR_RATIO 1.2121212121212121212121212121212121212121212121212121212121212121212121212121212121212121212121212121212121
//old
//#define ARM_FLOOR_POSITION -30 * GEAR_RATIO 			//Arm encoder positions
#define ARM_SHOOT_POSITION -540 * GEAR_RATIO
//#define ARM_CLIMB_POSITION -575 * GEAR_RATIO
//#define ARM_TRAVERSE_POSITION -290 * GEAR_RATIO

#define ARM_RESET_TOLERENCE .05
//new
#define ARM_FLOOR_POSITION -101.5 * GEAR_RATIO 			//Arm encoder positions
//#define ARM_SHOOT_POSITION -652.25 * GEAR_RATIO
#define ARM_CLIMB_POSITION -787.5 * GEAR_RATIO
#define ARM_TRAVERSE_POSITION -335.25 * GEAR_RATIO

#define ARM_FLOOR_RESET_POSITION -101.5 * GEAR_RATIO 			//Arm encoder positions
#define ARM_SHOOT_RESET_POSITION -652.25 * GEAR_RATIO
#define ARM_CLIMB_RESET_POSITION -787.5 * GEAR_RATIO
#define ARM_TRAVERSE_RESET_POSITION -335.25 * GEAR_RATIO
															//4.5  5.454
#define WINCH_PORT_POSITION 5950
#define WINCH_FASTCLIMB_POSITION 8350
#define WINCH_ZERO_POSITION 0
#define WINCH_CLIMB_POSITION -20340

#define SHIFT_UP_SPEED 2900 				//Encoder speed(rate) limits for shifting gears
#define SHIFT_DOWN_SPEED 25250

#define R_TRAIN(x)r_motor1.x;r_motor2.x; 	//sets all the motors in each drive train or roller
#define L_TRAIN(x)l_motor1.x;l_motor2.x;
#define ROLLERS(x)roller7.x;roller8.x;


class Robot: public SampleRobot
{
	Joystick l_stick;
	Joystick r_stick;
	Joystick a_stick;

	CANTalon l_motor1;
	CANTalon l_motor2;
	CANTalon r_motor1;
	CANTalon r_motor2;
	CANTalon winch5;
	CANTalon a_motor6;
	CANTalon roller7;
	CANTalon roller8;
	CANTalon a_motor9;

	Compressor cmprssr;

	std::shared_ptr<NetworkTable> table; 	//holds GRIP vision tracking values
	std::vector<double> centerX; 			//values from the NetworkTable
	std::vector<double> centerY;
	std::vector<double> area;

	Encoder armEncoderLeft;
	Encoder r_drive,l_drive; 				//Left and right motor encoders

	Solenoid shooter,shift;
	Solenoid armLock;

	float armPosition; 						//Keeps track of the arm position
	float armSetPoint;
	float winchPosition;
	float winchSetPoint;
	bool b_shift=1; 						//Whether or not in high gear (Solenoid on/off)
	double avg_speed;
	double avgArmPos;
	double preArmPos;

	unsigned char autonMode;

	bool resetLowBar = 0;

//	AHRS ahrs;

	Timer shift_timer;

	const static unsigned char autoModes = 4;
	char* autonomousMode[autoModes] = {
			"Nothing",
			"Low Bar",
			"Rock Wall",
			"Moat"
	};

	enum {
		Nothing,
		LowBar,
		RockWall,
		Moat
	};

	typedef enum {
		Floor,
		Shoot,
		Climb,
		Traverse
	}ArmPosition;

	typedef enum {
		Port,
		Zero,
		Climber,
		ClimbDown
	}WinchPosition;

public:
	Robot() :
			l_stick(0),
			r_stick(1),
			a_stick(2),
			l_motor1(1),
			l_motor2(2),
			r_motor1(3),
			r_motor2(4),
			winch5(5),
			a_motor6(6),
			roller7(7),
			roller8(8),
			a_motor9(9),
			armEncoderLeft(1,0,Encoder::k4X), 	//Encoder A channel on port 0, Encoder B channel on port 1
			r_drive(2,3,Encoder::k4X),
			l_drive(4,5,Encoder::k4X),
			shooter(1),
			shift(0),
			armLock(2)

	{
		armPosition = 0.0;
		armSetPoint = 0.0; 											//Start the setpoint at 0
		winchPosition = 0.0;
		winchSetPoint = 0.0;

		a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
		a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);

		winch5.SetEncPosition(0);
		winch5.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);

		roller7.SetVoltageRampRate(24);
		roller8.SetVoltageRampRate(24);

	//	ahrs = new AHRS(SerialPort::kMXP);
	}


	void Autonomous()
	{
		StopMotors();

		l_drive.Reset();
		r_drive.Reset();
		resetLowBar = 0;

		/*std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);

		if(autoSelected == autoNameCustom){
			//Custom Auto goes here

		} else {
			//Default Auto goes here

		}*/
			l_drive.Reset();
			r_drive.Reset();
			switch(autonMode) {
			case Nothing:
				return;
			case LowBar:
				LowBarAuto();
				break;
			case RockWall:
				RockWallAuto();
				break;
			case Moat:
				MoatAuto();
				break;
			//}
		}
	}

	void LowBarAuto()
	{
		DriveStraight(8000,1);
	}

	void RockWallAuto()
	{
		DriveStraight(8000,1);
	}

	void MoatAuto()
	{
		DriveStraight(8000,1);
	}

	void OperatorControl()
	{
#ifdef DEBUG
		PowerDistributionPanel pdp; 				//Initializes the pdp if DEBUG is not commented out at the top
#endif // DEBUG

		//StopMotors(); 								//Stops all motors before beginning to drive
		SmartDashboard::PutString("DB/String 3", std::to_string(armEncoderLeft.GetDistance()));
		preArmPos = armEncoderLeft.GetDistance();
		while (IsOperatorControl() && IsEnabled())
		{

#ifdef DEBUG
#ifdef THROTTLE
			ROLLERS(Set(a_stick.GetThrottle()));	//If both DEBUG and THROTTLE are defined,
													//then set the roller speed to the throttle
#endif // THROTTLE

													//If DEBUG, then display PDP and Talon values in SmartDahsboard Strings
			SmartDashboard::PutString("DB/String 5",
					(string)"PDP 1: "  + std::to_string(pdp.GetCurrent(1)) + (string)"A");
			SmartDashboard::PutString("DB/String 6",
					(string)"PDP 14: " + std::to_string(pdp.GetCurrent(14)) + (string)"A");

			SmartDashboard::PutString("DB/String 8",
					(string)"Talon 6: " + std::to_string(a_motor6.GetOutputCurrent()) + (string)"A");
			SmartDashboard::PutString("DB/String 9",
					(string)"Talon 9: " + std::to_string(a_motor9.GetOutputCurrent()) + (string)"A");

			SmartDashboard::PutString("DB/String 1", (string)"Left Encoder: " + std::to_string(l_drive.GetRate()).c_str());
			SmartDashboard::PutString("DB/String 2", (string)"Right Encoder: " + std::to_string(-r_drive.GetRate()).c_str());

			SmartDashboard::PutString("DB/String 3", (string)"ArmEncoder: " + std::to_string(armEncoderLeft.GetDistance()));
			SmartDashboard::PutString("DB/String 7", (string)"WincEncoder: " + std::to_string(winch5.GetEncPosition()));

#endif // DEBUG
			SmartDashboard::PutString("DB/String 3", (string)"ArmEncoder: " + std::to_string(armEncoderLeft.GetDistance()));
			shooter.Set(a_stick.GetRawButton(1)); //If DEBUG, then shoot on a_stick trigger press

			table = NetworkTable::GetTable("GRIP/myContoursReport");
		    centerX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
		    centerY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
		    area = table->GetNumberArray("area", llvm::ArrayRef<double>());

		    if(a_stick.GetRawButton(2))
		    {
		    	DriveWinchToPosition(Port);
		    }
		    else if(a_stick.GetRawButton(3))
		    {
		    	DriveWinchToPosition(Zero);
		    }
		    else if(a_stick.GetRawButton(5))
		    {
		    	winch5.Set(a_stick.GetY());
		    }
		    else if(a_stick.GetRawButton(6))
		    {
		    	DriveWinchToPosition(Climber);
		    }
		    else if(a_stick.GetRawButton(7))
		    {
		    	DriveWinchToPosition(ClimbDown);
		    }
		    else
		    {
		    	DriveWinchToPosition(Zero);
		    }



			//if(l_stick.GetRawButton(1))
			//	TrackTarget();

			//else if (r_stick.GetRawButton(1))
			//	TrackTargetPivot();

		/*	else if (l_stick.GetRawButton(2))
			{
				FollowMe();
			} */



			///else
				TankDrive();
/*
#ifdef ARM_JOYSTICK_CONTROL
			a_motor6.Set(a_stick.GetY());		//If DEBUG, then control arm with a_stick
			a_motor9.Set(-a_stick.GetY());
#endif // ARM_JOYSTICK_CONTROL
*/
			//Roller control
		/*	if(a_stick.GetRawButton(7) || a_stick.GetRawButton(12))
			{
				ROLLERS(Set(1));
			}
			else */ if(a_stick.GetRawButton(8))
			{
				ROLLERS(Set(-1));
			}
			else
			{
				ROLLERS(Set(0));
			}

			//Arm position control
			if(a_stick.GetRawButton(11))
			{
				ROLLERS(Set(-1));
				DriveArmToPosition(Floor);
			}
			else if(a_stick.GetRawButton(12))
			{
				ROLLERS(Set(1));
				DriveArmToPosition(Shoot);
			}
			else if(a_stick.GetRawButton(9))
			{
				DriveArmToPosition(Climb);
			}
			else if(a_stick.GetRawButton(10))
			{
				a_motor6.Set(0.0);
				a_motor9.Set(0.0);
			}
			else //if(a_stick.GetRawButton(10))
			{
				DriveArmToPosition(Traverse);
			}

			if(l_stick.GetRawButton(6))
				shift.Set(0);
			else
				AutoShift();

			Wait(0.005);						// wait for a motor update time
		}
	}


	void AutoShift() //Automatically shifts gears based on average speed of motors
	{

		avg_speed = fabs((l_drive.GetRate())-r_drive.GetRate())/2;
		if(avg_speed>SHIFT_UP_SPEED)
			shift.Set(1);
		else if(avg_speed<SHIFT_DOWN_SPEED)
			shift.Set(0);
	}


	void StopMotors() //Sets all motors to 0
	{
		l_motor1.Set(0);
		l_motor2.Set(0);
		r_motor1.Set(0);
		r_motor2.Set(0);
		winch5.Set(0);
		a_motor6.Set(0);
		roller7.Set(0);
		roller8.Set(0);
		a_motor9.Set(0);

		shooter.Set(0);
	}

	void DriveArmToPosition(ArmPosition setpoint, bool reset = 0) //Sets arm in position 'setpoint'
	{
		//First give the encoder a setpoint to drive to
		if(reset) {
			switch(setpoint) {
			case Floor:

				armSetPoint = ARM_FLOOR_POSITION;
				break;
			case Shoot:

				armSetPoint = ARM_SHOOT_POSITION;
				break;
			case Climb:
				armSetPoint = ARM_CLIMB_POSITION;
				break;
			case Traverse:
				armSetPoint = ARM_TRAVERSE_POSITION;
				break;
			default:
				std::cerr<<"Uninitialized arm position setting"<<endl;
			}
			DriveArmToPosition(setpoint,1);
			armLock.Set(0);
		}
		else
			switch(setpoint){
			case Floor:

				armSetPoint = ARM_FLOOR_POSITION;
				break;
			case Shoot:

				armSetPoint = ARM_SHOOT_POSITION;
				break;
			case Climb:
				armSetPoint = ARM_CLIMB_POSITION;
				break;
			case Traverse:
				armSetPoint = ARM_TRAVERSE_POSITION;
				break;
			default:
				std::cerr<<"Uninitialized arm position setting"<<endl;
			}
		armPosition = armEncoderLeft.GetDistance();
		/*avgArmPos = (armPosition + preArmPos) / 2;
		preArmPos = armPosition;
		armPosition = avgArmPos;*/
		float error = armPosition - armSetPoint;

#ifdef DEBUG
		SmartDashboard::PutString("DB/String 0", "ArmError: " + std::to_string(error));
#endif //DEBUG

		float motorOutputBeforeCbrt = (error/100);		//650
		float motorOutput;
		if(motorOutputBeforeCbrt < 0) //Make sure we don't take the sqrt of a negative number
		{
			motorOutputBeforeCbrt = -motorOutputBeforeCbrt;
			motorOutput = cbrt(motorOutputBeforeCbrt);
			motorOutput = -motorOutput;
		}
		else
		{
			motorOutput = cbrt(motorOutputBeforeCbrt);  // cbrt no multiplier
		}
		if(motorOutput < 0)
		{
			motorOutput = motorOutput/2;
		}

		if(armPosition > -125 && setpoint == Floor)
			{
			a_motor9.Set(0.0);
			a_motor6.Set(0.0);

			}
		else{
				a_motor9.Set(motorOutput);
				a_motor6.Set(-motorOutput);
		}
		if(fabs(motorOutput)<ARM_RESET_TOLERENCE && !reset)
			armLock.Set(1);
#ifdef DEBUG

		SmartDashboard::PutString("DB/String 4", (string)"Error " + std::to_string(motorOutput));
#endif //DEBUG
	}

	void DriveWinchToPosition(WinchPosition setpoint) //Sets winch in position 'setpoint'
	{
			switch(setpoint){
			case Port:
				winchSetPoint = WINCH_PORT_POSITION;
				break;
			case Zero:
				winchSetPoint = WINCH_ZERO_POSITION;
				break;
			case Climber:
				winchSetPoint = WINCH_CLIMB_POSITION;
				break;
			case ClimbDown:
				winchSetPoint = WINCH_FASTCLIMB_POSITION;
				break;
			default:
				std::cerr<<"Non-existant winch position setting"<<endl;
			}

			winchPosition = winch5.GetEncPosition();
			float error_w = winchPosition - winchSetPoint;
			float motorOutputBeforeCbrt;
			if(setpoint == ClimbDown)
			{
			 motorOutputBeforeCbrt = error_w / 250;
			}
			else
			{
			 motorOutputBeforeCbrt = error_w / 1500;
			}

			winch5.Set(motorOutputBeforeCbrt);
		}



	void GearShift() //Shifts gears based on a single button press
	{
		if(l_stick.GetRawButton(6))
		{
			shift.Set(0);
			b_shift=0;
		}
		//else if(!b_shift)
		if(l_stick.GetRawButton(7))
		{

			shift.Set(1);
			b_shift=1;
		}
	}

	void DriveStraight(int distance, bool forward)
		{
			if(forward)
				DriveStraightForward(distance);
			else DriveStraightBack(distance);
			/*int dir = forward? 1 : -1;
			int sign = (r_drive.GetDistance()+l_drive.GetDistance())/fabs(r_drive.GetDistance()+l_drive.GetDistance());
			double pos = sign * (fabs(r_drive.GetDistance())+fabs(l_drive.GetDistance()))/2;
			if((pos < distance) ^ forward)
			{
					if((fabs(r_drive.GetDistance())>fabs(l_drive.GetDistance())) ^ forward)
					{
							L_TRAIN(Set(-.5 * dir));
							R_TRAIN(Set(1 * dir));
					}
					else if((fabs(r_drive.GetDistance())<fabs(l_drive.GetDistance())) ^ forward)
					{
							L_TRAIN(Set(-1 * dir));
							R_TRAIN(Set(.5 * dir));
					}
					else
					{
							L_TRAIN(Set(-1));
							R_TRAIN(Set(1));
					}
			}
			else
			{
					L_TRAIN(Set(0));
					R_TRAIN(Set(0));
			}*/
		}
	void DriveStraightForward(int distance)
	{
		while(1)
		{
			int sign = (r_drive.GetDistance()+l_drive.GetDistance())/fabs(r_drive.GetDistance()+l_drive.GetDistance());
			double pos = sign * (fabs(r_drive.GetDistance())+fabs(l_drive.GetDistance()))/2;
			if((pos < distance))
			{
					if((fabs(r_drive.GetDistance())>fabs(l_drive.GetDistance())))
					{
							R_TRAIN(Set(.75));
							L_TRAIN(Set(-1));
					}
					else if((fabs(r_drive.GetDistance())<fabs(l_drive.GetDistance())))
					{
							R_TRAIN(Set(1));
							L_TRAIN(Set(-.75));
					}
					else
					{
							R_TRAIN(Set(1));
							L_TRAIN(Set(-1));
					}
			}
			else
			{
					L_TRAIN(Set(0));
					R_TRAIN(Set(0));
					return;
			}
		}
	}
	void DriveStraightBack(int distance)
	{
		while(1)
		{
			int sign = (r_drive.GetDistance()+l_drive.GetDistance())/fabs(r_drive.GetDistance()+l_drive.GetDistance());
			double pos = sign * (fabs(r_drive.GetDistance())+fabs(l_drive.GetDistance()))/2;
			if((pos < distance))
			{
					if((fabs(r_drive.GetDistance())<fabs(l_drive.GetDistance())))
					{
							R_TRAIN(Set(-.75));
							L_TRAIN(Set(1));
					}
					else if((fabs(r_drive.GetDistance())>fabs(l_drive.GetDistance())))
					{
							R_TRAIN(Set(-1));
							L_TRAIN(Set(.75));
					}
					else
					{
							R_TRAIN(Set(-1));
							L_TRAIN(Set(1));
					}
			}
			else
			{
					L_TRAIN(Set(0));
					R_TRAIN(Set(0));
					return;
			}
		}
	}


	void TankDrive() //Drives each train based on each joystick
	{
		R_TRAIN(Set(-r_stick.GetY()));
		L_TRAIN(Set(l_stick.GetY()));
	}


	void TrackTarget() //Turns towards any target in sight with only the right train
	{
		double distoff = 0.0;

		if(!centerX.empty())
		{
			distoff = 160 - centerX[0];

				if(centerX[0] < 140.00)
				{
					//l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(distoff/160 * TRACKMULT);
				}

				else if(centerX[0] > 180.00)
				{
					//l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(distoff/160 * TRACKMULT);
				}

				else if(centerX[0] > 140 && centerX[0] < 180)
				{
					l_motor1.Set(0);
					r_motor1.Set(0);
				}
		}
	}


	void TrackTargetPivot() //Turns towards any target in sight with both trains
	{
		double distoff = 0.0;

		if(!centerX.empty())
		{
			distoff = 160 - centerX[0];
				if(centerX[0] < 140.00)
				{
					l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(distoff/160 * TRACKMULT);
				}

				else if(centerX[0] > 180.00)
				{
					l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(distoff/160 * TRACKMULT);
				}

				else if(centerX[0] > 140 && centerX[0] < 180)
				{
					l_motor1.Set(0);
					r_motor1.Set(0);
				}
		}
	}


	void FollowMe() //Follows target in a straight line by measuring area of target
	{
		if (area[0] > 1000)
		{
			L_TRAIN(Set(0));
			R_TRAIN(Set(0));
		}

		else if (area[0] < 500)
		{
			L_TRAIN(Set(0.3));
			R_TRAIN(Set(-0.3));
		}

		else if (area[0] < 1000 && area[0] > 500)
		{
			L_TRAIN(Set(0.2));
			R_TRAIN(Set(-0.2));
		}

		else
		{
			L_TRAIN(Set(0));
			R_TRAIN(Set(0));
		}
	}


	void Disabled()
	{
		StopMotors();
		while(IsDisabled())
		{

			autonMode=0;
			for(int i=0;i<4;i++)
				autonMode+=pow(2,i)*(int)SmartDashboard::GetBoolean(((string)"DB/Button "+to_string(i)).c_str(),0);
			SmartDashboard::PutNumber("DB/Slider 0",autonMode);

			SmartDashboard::PutString("DB/String 0", (string)"ArmEncoder: " + std::to_string(armEncoderLeft.GetDistance()));
			SmartDashboard::PutString("DB/String 1", (string)"WincEncoder: " + std::to_string(winch5.GetEncPosition()));
			SmartDashboard::PutString("DB/String 2", (string)"Left Drive: " + std::to_string(l_drive.GetDistance()));
			SmartDashboard::PutString("DB/String 3", (string)"Right Drive: " + std::to_string(r_drive.GetDistance()));
			SmartDashboard::PutString("DB/String 4", "");

			SmartDashboard::PutString("DB/String 5",
					(string)"Autonomous: "+(string)((autonMode<autoModes)?autonomousMode[autonMode]:"Error"));
			SmartDashboard::PutString("DB/String 6", "");
			SmartDashboard::PutString("DB/String 7", "");
			SmartDashboard::PutString("DB/String 8", "");
			SmartDashboard::PutString("DB/String 9", "");


		}
	}
};

START_ROBOT_CLASS(Robot)
