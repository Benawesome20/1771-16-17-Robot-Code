#include "WPILib.h"
#include<thread>
#include <Math.h>

#include"Buttons.h"
#include"Positions.h"
#include"enums.h"

using namespace std;

#define DEBUG 								//Debug mode, shows stats as well as other functions for testing
//#define THROTTLE							//Sets rollers to be controlled by the a_stick throttle
//#define ARM_JOYSTICK_CONTROL

#define TRACKMULT 1.1771 * 12				//Multiplier for vision tracking speed

#define CLIMB_WAIT_TIME 0.51771
#define AUTON_WAIT_TIME 4.01771

#define ARM_RESET_TOLERENCE .05

#define SHIFT_UP_SPEED 2900 				//Encoder speed(rate) limits for shifting gears
#define SHIFT_DOWN_SPEED 2200

#define ROLLER_VOLTAGE_RAMP 25 // last test: 15		before mod: 32

#define R_TRAIN(x)r_motor1.x;r_motor2.x; 	//sets all the motors in each drive train or roller
#define L_TRAIN(x)l_motor1.x;l_motor2.x;
#define ROLLERS(x)roller7.x;roller8.x;ShootTimer.Stop();ShootTimer.Reset();ShootTimer.Start();

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
	std::vector<double> width;

	Encoder armEncoderLeft;
	Encoder r_drive,l_drive; 				//Left and right motor encoders

	Solenoid shooter,shift;
	Solenoid armLock;
	Relay vrmSpike;


	float autonDistanceDriven;
	float distanceDrivenInNewAuton;
	float currentDistance;
	float armCameraSetPoint;
	float armPosition; 						//Keeps track of the arm position
	float armSetPoint;
	float winchPosition;
	float winchSetPoint;
	bool b_shift=1; 						//Whether or not in high gear (Solenoid on/off)
	double avg_speed;
	double avgArmPos;
	bool isArmResetPos = 0;
	bool reachedPoint = 0;

	/* Auton booleans */
	bool needsToGoBack = false;
	bool needsToStop = false;
	bool doneForward = false;
	bool resetEncoders = false;
	float backwardsDistance = 0.0;
	bool firstRun = true;
	unsigned char autonMode;

	bool resetLowBar = 0;
	bool hasReachedSetPoint = false;

	DigitalInput ClimbComplete;

	bool climbing = false;
	bool preclimb = false;
	bool climbed = false;
	bool lowGoalFix = false;
	
	Timer climb_timer;
	Timer auton_timer;
	Timer autonTimer2;
	Timer ShootTimer;
	
	thread* armStartAuto;
	
#ifdef DEBUG
	PowerDistributionPanel pdp; 				//Initializes the pdp if DEBUG is not commented out at the top
#endif // DEBUG

	const static unsigned char autoModes = 6;
	char* autonomousMode[autoModes] = {
			"Nothing",
			"Low Bar",
			"Rock Wall",
			"PortAutoMode",
			"Low Bar Shoot",
			"FastRockWall"
	};

	typedef enum AutoMode;

	typedef enum ArmPosition;

	typedef enum WinchPosition;

	ArmPosition preArmPos = Shoot;
	ArmPosition currentArmPos = Traverse;
	ArmPosition pickupFixer= Traverse;

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
			armLock(2),
			ClimbComplete(6),
			vrmSpike(7)
	{
		armPosition = 0.0;
		armSetPoint = 0.0; 											//Start the setpoint at 0
		winchPosition = 0.0;
		winchSetPoint = 0.0;
		distanceDrivenInNewAuton = 0;
		autonDistanceDriven = 0;
		a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
		a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
		r_motor1.SetControlMode(CANTalon::kVoltage);
		r_motor2.SetControlMode(CANTalon::kVoltage);
		l_motor1.SetControlMode(CANTalon::kVoltage);
		l_motor2.SetControlMode(CANTalon::kVoltage);
		/*r_motor1.SetVoltageRampRate(240);
		r_motor2.SetVoltageRampRate(240);
		l_motor1.SetVoltageRampRate(240);
		l_motor2.SetVoltageRampRate(240);*/
		winch5.SetEncPosition(0);
		winch5.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);

		roller7.SetVoltageRampRate(ROLLER_VOLTAGE_RAMP);
		roller8.SetVoltageRampRate(ROLLER_VOLTAGE_RAMP);
	}

	void Autonomous()
	{

		StopMotors();

		l_drive.Reset();
		r_drive.Reset();
		resetLowBar = 0;

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
		case PortAutoMode:
			PortAuto();
			break;
		case LowBarShoot:
			LowBarShootAuto();
			break;
		case FastRockWall:
			FastRockWallAuto();
		default:
			break;
		}
	}

	void LowBarAuto()
	{
		float distanceDriven = 0.0;
		float gyroAngle;
		bool hasDrivenForward = false;
		bool timerHasStarted = false;
		while(IsAutonomous() && IsEnabled())
		{

			if(firstRun)
			{
				armLock.Set(1);
				DriveArmToPosition(Floor);
				l_drive.Reset();
				r_drive.Reset();
				firstRun = false;
				Wait(1);
			}
			if(!hasDrivenForward)
			{
				if(distanceDriven < 8000)
				{
					distanceDriven = (fabs(l_drive.GetDistance()) + fabs(r_drive.GetDistance()) ) / 2;
					DriveStraight(8000, 1, .5);
				}
				if(distanceDriven > 8000)
				{
					hasDrivenForward = true;
				}
			}
			if(hasDrivenForward)
			{
				StopMotors();
			}
			Wait(.005);
		}

	}

	void LowBarShootAuto()
	{
		float distanceDriven = 0.0;
		bool hasDrivenForward = false;
		bool needsToDriveMore = false;
		bool mustStop = false;
		bool timerHasStarted = false;
		bool timer2HasStarted = false;
		
		armLock.Set(1);
		
		l_drive.Reset();
		r_drive.Reset();
		
		firstRun = false;
		
		if(IsAutonomous() && IsEnabled())
		    while(DriveArmToPosition(Floor)); // Repeat until in position

		while(IsAutonomous() && IsEnabled())
		{
			if(!table.get() == NULL)
			{
				table = NetworkTable::GetTable("GRIP/myContoursReport");
				centerX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
				area = table->GetNumberArray("area", llvm::ArrayRef<double>());
				if(!centerX.empty())
				{
					double centerXD = centerX[0];
					double areaD = area[0];
				}
			}
			
			if(!hasDrivenForward)
			{
				armLock.Set(1);
				DriveArmToPosition(Floor);
				if(distanceDriven < 7000)
				{
					distanceDriven = (fabs(l_drive.GetDistance()) + fabs(r_drive.GetDistance()) ) / 2;
					DriveStraight(7000, 1, 1);
				}
				if(distanceDriven > 7000)
				{
					hasDrivenForward = true;
				}
			}
			if(hasDrivenForward && !needsToDriveMore) //Turn to the right
			{
				armCameraSetPoint = -450;
				ROLLERS(Set(-1));
				if(!timer2HasStarted)
				{
					autonTimer2.Start();
					timer2HasStarted = true;
				}
				else
				{

				if(autonTimer2.Get() < 5)
				//Do this 3 times, so the arm can keep its position updated correctly]

					DriveArmToPosition(Camera_Setpoint);
					R_TRAIN(Set(.4*12));
					L_TRAIN(Set(-1*12));
					Wait(.05);

				}
				if(autonTimer2.Get() > 5)
				{
			    	needsToDriveMore = true;
				    mustStop = true;
				}
			}
			if(needsToDriveMore && !mustStop)
			{

			}
			if(mustStop)
			{
				armCameraSetPoint = -450;
				if(!timerHasStarted)
				{
				    auton_timer.Start();
				}
				if(timerHasStarted)
				{
					if(auton_timer.Get() < AUTON_WAIT_TIME)
					{
						DriveArmToPosition(Camera_Setpoint);
						TrackTarget();
					}
					if(auton_timer.Get() > AUTON_WAIT_TIME)
					{
						shooter.Set(1);
						Wait(.5);
						DriveArmToPosition(Traverse);
					}
				}
			}
			Wait(.005);
		}
	}

	void FastRockWallAuto()
	{
	    doneForward = false;
	    
	    l_drive.Reset();
		r_drive.Reset();
	    
	    if(IsAutonomous() && IsEnabled())
		    while(DriveArmToPosition(Traverse)); // Repeat until in position
		    
		while(IsAutonomous() && IsEnabled())
		{
			DriveArmToPosition(Traverse);
			if((fabs(l_drive.GetDistance()) + fabs(r_drive.GetDistance()) ) / 2 < 8100)
			{
				R_TRAIN(Set(10));
				L_TRAIN(Set(-10));
				doneForward = true;
				DriveWinchToPosition(Zero);
			}
			else if(doneForward)
			{
				R_TRAIN(Set(0.0));
				L_TRAIN(Set(0.0));
				
				DriveWinchToPosition(Snap);
			}
			Wait(.005);
		}

		/*while(IsAutonomous() && IsEnabled())
					{//DriveArmToPosition(Traverse);
			SmartDashboard::PutString("DB/String 4", (string)"Auton Distance " + std::to_string(autonDistanceDriven));
						DriveArmToPosition(Traverse);
						if(autonDistanceDriven < 10000)
						{
						DriveStraight(2000,1);
						}
						else
						{
							StopMotors();
						}
						Wait(.005);
					}
					*/
	}

	void RockWallAuto()
	{
		while(IsAutonomous() && IsEnabled())
		{
			DriveArmToPosition(Traverse);
			if(firstRun)
			{
				l_drive.Reset();
				r_drive.Reset();
				firstRun = false;
				Wait(1);
			}
			if(  (fabs(l_drive.GetDistance()) + fabs(r_drive.GetDistance()) ) / 2 < 8100 && (!needsToGoBack))
			{
				R_TRAIN(Set(6));
				L_TRAIN(Set(-6));
				doneForward = true;
			}
			else if(doneForward && !resetEncoders)
			{
				resetEncoders = true;
				needsToGoBack = true;
				R_TRAIN(Set(0.0));
				L_TRAIN(Set(0.0));
				l_drive.Reset();
				r_drive.Reset();
				Wait(.1);
			}
			if(needsToGoBack)
			{
				backwardsDistance = (fabs(l_drive.GetDistance()) + fabs(r_drive.GetDistance()) ) / 2;
				if(backwardsDistance < 6100)
				{
					R_TRAIN(Set(-.7 * 12));
					L_TRAIN(Set(.7 * 12));
				}
				else
				{
					R_TRAIN(Set(0.0));
					L_TRAIN(Set(0.0));

					DriveWinchToPosition(Snap);
				}
			}
			Wait(.005);
		}

		/*while(IsAutonomous() && IsEnabled())
					{//DriveArmToPosition(Traverse);
			SmartDashboard::PutString("DB/String 4", (string)"Auton Distance " + std::to_string(autonDistanceDriven));
						DriveArmToPosition(Traverse);
						if(autonDistanceDriven < 10000)
						{
						DriveStraight(2000,1);
						}
						else
						{
							StopMotors();
						}
						Wait(.005);
					}
					*/
	}

	void PortAuto()
	{
		bool firstRun = true;

			while(IsAutonomous() && IsEnabled())
			{
				DriveArmToPosition(Traverse);
						if(firstRun)
						{
							auton_timer.Start();
							firstRun = false;
						}
						if(auton_timer.Get() < 12)
						{
							R_TRAIN(Set(6));
							L_TRAIN(Set(-6));
						}
						else
						{
							StopMotors();
						}
						Wait(.005);
			}
	}

	void OperatorControl()
	{


		StopMotors(); 								//Stops all motors before beginning to drive
		climbed = false;
		while (IsOperatorControl() && IsEnabled())
		{
			if(climbed) 	// Holofect sensor input
				ClimbUp();
			else {
#ifdef DEBUG	//If DEBUG, then display PDP and Talon values in SmartDahsboard Strings

#ifdef THROTTLE
			ROLLERS(Set(a_stick.GetThrottle()));	//If both DEBUG and THROTTLE are defined,
													//then set the roller speed to the throttle
#endif // THROTTLE

			PrintString();

#endif // DEBUG
				if(l_stick.GetRawButton(6)) {
					//if(ShootTimer.Get() > 3)
						shooter.Set(1); //If DEBUG, then shoot on a_stick trigger press
				}
				else {
					shooter.Set(0);
				}

				if(!table.get() == NULL)
				{
					table = NetworkTable::GetTable("GRIP/myContoursReport");
					centerX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
					area = table->GetNumberArray("area", llvm::ArrayRef<double>());
					if(!centerX.empty())
					{
						double centerXD = centerX[0];
						double areaD = area[0];

						SmartDashboard::PutString("DB/String 6", (string)"CenterX: " + std::to_string(centerXD));
						SmartDashboard::PutString("DB/String 7", (string)"CenterD: " + std::to_string(areaD));
					}
				}

				if(!A2)
					preclimb = false;
				if(A2 || climbing)
					if(A1)
						ClimbUp();
					else PreClimb();
				else if(A3)
					DriveWinchToPosition(Port);
				else if(A6)
					DriveWinchToPosition(Climber);
				else if(A7)
					DriveWinchToPosition(ClimbDown);
				else
					DriveWinchToPosition(Zero);

				if(R11) {
					vrmSpike.Set(Relay::kForward);
					armCameraSetPoint = -473;
					DriveArmToPosition(Camera_Setpoint);
					TankDrivePrecise();
					//TrackTarget();
				}
				/*else if(r_stick.GetRawButton(9))
				{
					armCameraSetPoint = -450;
					DriveArmToPosition(Camera_Setpoint);
					TrackTarget();
				}*/
				else if(!(r_stick.GetRawButton(9) || r_stick.GetRawButton(10)))
					TankDrive();

				if(!(A11 || R11 || A12) { // If Not PickUp, Shoot, or Tracking
					ROLLERS(Set(0));
				}

				if(!(R11 || r_stick.GetRawButton(9) || r_stick.GetRawButton(10)))
				{
					vrmSpike.Set(Relay::kOff);
				}
				//else if (r_stick.GetRawButton(1))
				//	TrackTargetPivot();

			/*	else if (l_stick.GetRawButton(2))
				{
					FollowMe();
				} */

	\
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
				else */

				//Arm position control
				if(!preclimb) {
					climb_timer.Stop(); // Problem Prevention
					climb_timer.Reset(); // Problem Prevention
					if(A11)
					{
                        ROLLERS(Set(1));
                        armLock.Set(1);
                        
					    if(reachedPoint || pickupFixer != Traverse) {
					        DriveArmToPosition(Floor);
					        pickupFixer = Floor;
					    }
					    else
					        reachedPoint = !DriveArmToPosition(Travers_Pickup);
					}
					else if(A12)
					{
						ROLLERS(Set(-1));
						DriveArmToPosition(Shoot);
						pickupFixer = Shoot;
					}
					else if(A10)
					{
						a_motor6.Set(0.0);
						a_motor9.Set(0.0);
					}
					else if(A9)
					{
                        ROLLERS(Set(-1));
                        armLock.Set(1);
                        
					    if(reachedPoint || pickupFixer != Traverse) {
					        DriveArmToPosition(Low_Goal);
					        pickupFixer = Low_Goal;
					    }
					    else
					        reachedPoint = !DriveArmToPosition(Travers_Pickup);
					}
					else if(R11)
					{
						vrmSpike.Set(Relay::kForward);
						//ROLLERS(Set(1));
						armCameraSetPoint = -473;
						DriveArmToPosition(Camera_Setpoint);
						TrackTarget();
					//	TankDrivePrecise();
					}
					else if(r_stick.GetRawButton(9))
					{
						vrmSpike.Set(Relay::kForward);
						armCameraSetPoint = -452;
					DriveArmToPosition(Camera_Setpoint);
							//TrackTarget();
					TankDrivePrecise();
					}
					else if(r_stick.GetRawButton(10))
					{
						vrmSpike.Set(Relay::kForward);
						armCameraSetPoint = -473;
						DriveArmToPosition(Camera_Setpoint);
						TrackTarget();
					}
					else// if(a_stick.GetRawButton(10))
					{
						//DriveArmToPosition(Traverse, 1);
						DriveArmToPosition(Traverse);
						lowGoalFix = false;
						pickupFixer = Traverse;
						armLock.Set(0);
					}

				}
				/*if(l_stick.GetRawButton(6))
					shift.Set(0);
				else
					AutoShift();
					*/
					AutoShift();
			}
			Wait(0.005);						// wait for a motor update time

			//if(a_stick.GetRawButton())
		}
	}

	void PreClimb()
	{
		climbing = false;
		preclimb = true;
		climb_timer.Stop();
		climb_timer.Reset();

		DriveArmToPosition(Pre_Climb);
		DriveWinchToPosition(Climber);
	}

	void ClimbUp()
	{
		DriveArmToPosition(Climb);
		if(!climbing) {
			climb_timer.Start();
			climbing = true;
		}
		if(climb_timer.Get() > CLIMB_WAIT_TIME) {
			if(!ClimbComplete.Get())
				climbed = true;
			if(climbed) {	// Holofect sensor input
				winch5.Set(0);
			}
			else
			{
				winch5.Set(-1);
			}
			cout<<"timer"<<endl;

		}
		SmartDashboard::PutNumber("DB/Slider 2",climb_timer.Get());
	}

	/**
	 * Automatically shifts gears based on average speed of motors
	 */
	void AutoShift()
	{
		avg_speed = fabs((l_drive.GetRate() - r_drive.GetRate())) / 2;
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

	bool DriveArmToPosition(ArmPosition setpoint, double tolerence = ARM_RETURN_TOLERENCE) //Sets arm in position 'setpoint'
	{

		preArmPos = currentArmPos;
		currentArmPos = setpoint;
		if(preArmPos != setpoint)
			hasReachedSetPoint = false;
		switch(setpoint){
		case Floor:
			armSetPoint = ARM_FLOOR_POSITION;
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Coast);
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Coast);
			break;
		case Shoot:
			armSetPoint = ARM_SHOOT_POSITION;
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
			break;
		case Climb:
			armSetPoint = ARM_CLIMB_POSITION;
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
			break;
		case Traverse:
			armSetPoint = ARM_TRAVERSE_TO_PICKUP; // ARM_TRAVERSE_POSITION
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
			break;
		case Traverse_Pickup:
			armSetPoint = ARM_TRAVERSE_TO_PICKUP;
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
			break;
		case Camera_Setpoint:
			armSetPoint = armCameraSetPoint;
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
			break;
		case Slider_Pos:
			armSetPoint = SmartDashboard::GetNumber("DB/Slider 1", 500);
			break;
		case Pre_Climb:
			armSetPoint = PRE_CLIMB_POSITION;
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
			break;
		case Portculus:
			armSetPoint = ARM_PORT_POSITION;
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
			break;
		case Low_Goal:
			armSetPoint = ARM_LOW_GOAL_POSITION;
			a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
			a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
			break;
		default:
			std::cerr<<"Uninitialized arm position setting"<<endl;
		}
		armPosition = armEncoderLeft.GetDistance();
		float error = armPosition - armSetPoint;

		if(setpoint == Traverse)
			pickupFixer = Traverse;

		float motorOutputBeforeCbrt = (error/125);		//650
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

		if(setpoint == Traverse || setpoint == Low_Goal) {
			if(fabs(error) < 5)
			{
				lowGoalFix = true;
				hasReachedSetPoint = true;
			}
		}

		if(armPosition > -125 && setpoint == Floor)
			{
			a_motor9.Set(0.0);
			a_motor6.Set(0.0);
		}
		else if(setpoint == Traverse_Pickup) // Pull up quickly
		{
			a_motor9.Set(1.0);
			a_motor6.Set(-1.0);
		}
		else if(setpoint == Traverse && hasReachedSetPoint)
		{
			a_motor9.Set(-.05);
			a_motor6.Set(.05);
		}
		else {
			a_motor9.Set(motorOutput);
			a_motor6.Set(-motorOutput);
		}


#ifdef DEBUG
		SmartDashboard::PutString("DB/String 4", (string)"Arm Pos Error: " + std::to_string(motorOutput));
#endif //DEBUG

        return !(fabs(error) < tolerence)
	}

	bool DriveWinchToPosition(WinchPosition setpoint, double tolerence = WINCH_RETURN_TOLERENCE) //Sets winch in position 'setpoint'
	{
		if(!climbing || !climbed) {
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
			case Snap:
				winchSetPoint = WINCH_SNAP_POSITION;
				break;
			default:
				std::cerr<<"Non-existant wench position setting"<<endl;
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
			
			return !(fabs(error_w) < tolerence);
		}
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

	void DriveStraight(int distance, bool forward, double multiplier = 1)
	{
	//ROLLERS(Set(1));
		multiplier *= 12;
		if(forward)
			DriveStraightForward(distance, multiplier);
		else DriveStraightBack(distance, multiplier);
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

	void DriveStraightForward(int distance, double modifier = 1)
	{
		bool stopped = 0;
	//	ROLLERS(Set(1));
	//	while(IsAutonomous() && IsEnabled() && !stopped)
	//	{
			int sign = (r_drive.GetDistance()+l_drive.GetDistance() )/ ( fabs(r_drive.GetDistance()+l_drive.GetDistance()) ) ;
			double pos = sign * (fabs(r_drive.GetDistance())+fabs(l_drive.GetDistance()))/2;
			SmartDashboard::PutString("DB/String 8", (string)"AutonDist " + std::to_string(autonDistanceDriven));
			if((pos < distance))
			{
					if((fabs(r_drive.GetDistance())>fabs(l_drive.GetDistance())))
					{
							R_TRAIN(Set(.7 * modifier));
							L_TRAIN(Set(-1 * modifier));
					}
					else if((fabs(r_drive.GetDistance())<fabs(l_drive.GetDistance())))
					{
							R_TRAIN(Set(1 * modifier));
							L_TRAIN(Set(-.7 * modifier));
					}
					else
					{
							R_TRAIN(Set(1 * modifier));
							L_TRAIN(Set(-1 * modifier));
					}
			}
			else
			{
					L_TRAIN(Set(0));
					R_TRAIN(Set(0));
					stopped = 1;
			}
			autonDistanceDriven += (fabs(r_drive.GetDistance()) + fabs(l_drive.GetDistance()) )/ 2;
		//}
	}

	void DriveStraightBack(int distance, double modifier = 1)
	{
		bool stopped;
		int sign = (r_drive.GetDistance()+l_drive.GetDistance() )/ ( fabs(r_drive.GetDistance()+l_drive.GetDistance()) ) ;
		double pos = sign * (fabs(r_drive.GetDistance())+fabs(l_drive.GetDistance()))/2;
		SmartDashboard::PutString("DB/String 6", (string)"AutonDist " + std::to_string(autonDistanceDriven));
		if((pos < distance))
		{
			if((fabs(r_drive.GetDistance())<fabs(l_drive.GetDistance())))
			{
				R_TRAIN(Set(-1 * modifier));
				L_TRAIN(Set(.7 * modifier));
			}
			else if((fabs(r_drive.GetDistance())>fabs(l_drive.GetDistance())))
			{
				R_TRAIN(Set(-.7 * modifier));
				L_TRAIN(Set(1 * modifier));
			}
			else
			{
				R_TRAIN(Set(-1 * modifier));
				L_TRAIN(Set(1 * modifier));
			}
		}
		else
		{
			L_TRAIN(Set(0));
			R_TRAIN(Set(0));
			stopped = 1;
		}
		autonDistanceDriven += (fabs(r_drive.GetDistance()) + fabs(l_drive.GetDistance()) )/ 2;
	}

	void TankDrive() //Drives each train based on each joystick
	{
		R_TRAIN(Set(-r_stick.GetY()*12));
		L_TRAIN(Set(l_stick.GetY()*12));
	}

	void TankDrivePrecise() //Drives each train based on each joystick
	{
		R_TRAIN(Set((-r_stick.GetY()*12) * .7));
		L_TRAIN(Set((l_stick.GetY()*12) * .7));
	}

	void TrackTarget() //Turns towards any target in sight with only the right train
	{
		double distoff = 0.0;


		if(!centerX.empty())
		{
			distoff = 124 - centerX[0]; //160 is half of x resolution on camera
			float turnSpeed = 0.0;
			turnSpeed = (distoff/60 * TRACKMULT) /* * 12*/;
			if(fabs(turnSpeed) > 4)
			{
				if(turnSpeed < 0)
				{
					turnSpeed = -4;
				}
				if(turnSpeed > 0)
				{
					turnSpeed = 4;
				}
			}
			SmartDashboard::PutString("DB/String 7", (string)"TurnSpeed: " + std::to_string(turnSpeed));

			r_motor1.Set(turnSpeed);
			r_motor2.Set(turnSpeed);
			l_motor1.Set(turnSpeed);
			l_motor2.Set(turnSpeed);



				/*if(centerX[0] < 140.00)
				{
					turnSpeed = distoff/100 * TRACKMULT;
					if(fabs(turnSpeed) > .5)
						turnSpeed = .5;
					//l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(turnSpeed);
					r_motor2.Set(turnSpeed);
					l_motor1.Set(turnSpeed);
					l_motor2.Set(turnSpeed);
				}

				else if(centerX[0] > 180.00)
				{
					turnSpeed = distoff/100 * TRACKMULT;
					if(fabs(turnSpeed) > .5)
					turnSpeed = .5;
					//l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(-turnSpeed);
					r_motor2.Set(-turnSpeed);
					l_motor1.Set(-turnSpeed);
					l_motor2.Set(-turnSpeed);
				}*/
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
					l_motor1.Set(distoff/100 * TRACKMULT);
					r_motor1.Set(distoff/100 * TRACKMULT);
				}

				else if(centerX[0] > 180.00)
				{
					l_motor1.Set(distoff/100 * TRACKMULT);
					r_motor1.Set(distoff/100 * TRACKMULT);
				}

				else if(centerX[0] > 140 && centerX[0] < 180)
				{
					l_motor1.Set(0);
					r_motor1.Set(0);
				}
		}
	}

	void Test()
	{
		while(IsTest() && IsEnabled())
		{
		/*	if(a_stick.GetRawButton(1))
			{
				armLock.Set(1);
			}
			else
			{
				armLock.Set(0);
			}
			*/
			if(L8)
			{
				ROLLERS(Set(1));
			}
			else if(L9)
			{
				ROLLERS(Set(-1));
			}
			else
			{
				ROLLERS(Set(0));
			}

			// Winch Manual Control
			winch5.Set(a_stick.GetY());

			shooter.Set(l_stick.GetRawButton(1));

			//DriveArmToPosition(Slider_Pos);

			PrintString();
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

			PrintString();
			SmartDashboard::PutString("DB/String 5",
					(string)"Auto: "+(string)((autonMode<autoModes)?autonomousMode[autonMode]:"Error"));

			Wait(.005);
		}
	}

	float getCameraSetPoint(double centerMass)
	{
		float distanceTable[] =  { 200, 180, 166, 154, 143, 132, 123, 115, 108,102 };
		float rpmTable[] = { -1000, -1200, -1250, -1450, -1550, -1650, -1771, -1900,-2000,};
		if(centerMass == 0)
			centerMass = 1;
		/*
		  float distanceTable[] = { 200, 180, 166, 154, 143, 132, 123, 115, 108,102 };
		  float rpmTable[] = { 2530, 2520, 2520, 2590, 2650, 2760, 2770, 2820,2920, 3020 };
		*/
		//rpm calculation and interpolation

		/**
		 * distance between the right target - the left target
		 * the higher number the closer we are
		 * printf("CurrentDistance: %f \n", currentDistance);
		 * this if block tests the 2 extreme values we could encounter, 200 being closest.
		 */
		currentDistance = fabs(centerMass);
		if (currentDistance > 200) {
			armCameraSetPoint = -1000;
		} else if (currentDistance < 102) {
			armCameraSetPoint = -1500;
		}

		//if not at the actual positions, loop through
		float valueToReturn;
		int position;

		for (int i = 0; i <= 9; i++) {
			if ((currentDistance <= distanceTable[i])
					&& (currentDistance >= distanceTable[i + 1])) {
				//assign a "position" to be used later on
				//200,180,166,154,143,132,123,115,108,102
				position = i;
			}
		}

		//here is where position is used, for the linear interp.
		valueToReturn = (((rpmTable[position] - rpmTable[position + 1])
				/ (distanceTable[position] - distanceTable[position + 1]))
				* (currentDistance - distanceTable[position + 1]))
				+ rpmTable[position + 1];
		SmartDashboard::PutString("DB/String 7",(string)"CameraSetpoint: " + std::to_string(valueToReturn));
		return valueToReturn;
	}

	void PrintString()
	{
		SmartDashboard::PutString("DB/String 0",(string)"ArmEnc: " + std::to_string(armEncoderLeft.GetDistance()));
		SmartDashboard::PutString("DB/String 1", (string)"WincEnc: " + std::to_string(winch5.GetEncPosition()));
		SmartDashboard::PutString("DB/String 2",(string)"Left Enc: " + std::to_string(l_drive.GetRate()).c_str());
		//SmartDashboard::PutString("DB/String 3",(string)"Right Enc: " + std::to_string(-r_drive.GetRate()).c_str());
		//SmartDashboard::PutString("DB/String 4",(string)""); // see DriveArmToPosition(ArmPosition)
		//SmartDashboard::PutString("DB/String 5",(string)""); // see Disabled(void)
		//SmartDashboard::PutString("DB/String 6",(string)""); // see OperatorControl(void)
		SmartDashboard::PutString("DB/String 7",(string)""+ to_string(roller7.GetOutputCurrent())); // see OperatorControl(void)
		SmartDashboard::PutString("DB/String 8",(string)""+ to_string(roller8.GetOutputCurrent()));; // see DriveStrightForward(int, double = 1)
		SmartDashboard::PutString("DB/String 9",(string)"Holo: " + std::to_string(ClimbComplete.Get()));
	}
};

START_ROBOT_CLASS(Robot)
