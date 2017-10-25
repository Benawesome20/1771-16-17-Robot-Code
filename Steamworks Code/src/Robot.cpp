#include <iostream>
#include <memory>
#include <string>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <WPILib.h>
#include <CANTalon.h>
#include <fstream>

#include "Climber.h"
#include "Balls.h"
#include "Buttons.h"
#include "Definitions.h"
#include "Intake.h"
#include "Pixy.h"
#include "RoboBase.h"
#include "Transmission.h"
#include "Turret.h"
#include "Cartesian.h"

class Robot: public frc::IterativeRobot {
public:

	void RobotInit() {
		bot.StopMotors();
		bot.Reset();
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */


	void AutonomousInit() override {
		bot.Reset();
		autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
		bot.ZeroNavX();

		if (autoSelected == autoNameCustom) {
			bot.ZeroNavX();
		} else {
			// Default Auto goes here
		}
		bot.StartTimer();
		StartDataLog("auto.log");
	}

	int dist = 5000;

	//double right = SmartDashboard::GetNumber("DB/Slider 0", 0.3);
	//double left = SmartDashboard::GetNumber("DB/Slider 1", 0.3);
	void AutonomousPeriodic() {
		if (autoSelected == autoNameCustom) {
			/*if(bot.GetDistance() < 3000){
							bot.DriveXDistance(.3);
							SmartDashboard::PutString("DB/String 8", "If Is True");
						}else{
							SmartDashboard::PutString("DB/String 3", "If is False");
							bot.StopMotors();
						}*/

						if(bot.GetDistance() < 10000){ // Should travel 1.5 meters
							bot.DriveStraight(.3);
						}
						/*bool first_cross = 0;
						if(!first_cross){
							if(bot.GetDisplacementX() < 4 && bot.GetDistance() < 1000){ // Travel ~3.5 meters (or original distance calculation)
								bot.DriveStraight(.3);
							}else{
								bot.ResetDisplacement();
								first_cross = 1;
								bot.StopMotors();
							}
						}else{
							bool back = 0;
							if(!back && bot.GetDisplacementX() < 2){
								bot.DriveStraight(-.3);
							}else{
								back = 1;
								bot.ResetDisplacement();
							}

							if(back){
								bot.TurnAbsolute(45.0);
								if(bot.GetDisplacementTotal() < 1.5 && bot.GetDistance() < 1000)
									bot.DriveStraight(.3);
								else
									bot.StopMotors();
							}
						}
						*/
		}
		else {

			if(Timer::GetMatchTime() > 7.0){
				bot.DriveStraight(.3);
			}else{
				bot.StopMotors();
			}

		}


		bot.AddAccelerometerDistance();
		LogData("auto.log");
		PutNumbers();
	}

	void TestInit(){
		StartDataLog("test.log");
	}


	void TeleopInit() {
		StartDataLog("tele.log");
		bot.ZeroNavX();
		bot.ResetTimer();
		bot.StartTimer();
	}

	void TeleopPeriodic() {
		if (L7)
		{
			bot.StopMotors();
		}
		else if (L6)
		{
			bot.TrackHook();
		}
		else if (L8)
		{
			//bot.DriveXDistance(2000, 1, 1);
		}
		else if (L9)
		{
			bot.SetDist();
		}
		else
		{
			bot.TankDrive();
			bot.AutoShift();
		}

		if(T1 || R11)
		{
			//balls.turret.Fire();
			bot.SetCatch(0);
		}
		else
		{
			bot.SetCatch(1);
		}

		if(T5)
		{

		}
		else //if(T6)
		{
			balls.ManualClimb();
			//bot.ClearClimbEncoder();
		}

		if(T3)
		{
			balls.SetIntake(1);
		}
		else
		{
			balls.StopTurretFire();
			balls.SetIntake(0);
			balls.TurretStickDrive();
		}

		/***---=====<<<({|[ SHIMMY STUFF ]|})>>>=====---***/
		/*int back = 0, turn = 0;
		double angle = 1771;
		if(angle == 1771)
		 angle = bot.GetAbsoluteAngle();
		if(T5){
			bot.ShimmyLeft(&back, &turn, &angle);
		}else if(T6){
			//bot.ShimmyRight(&back, &turn, &angle);
		}*/


		bot.AddAccelerometerDistance();
		LogData("tele.log");
		PutNumbers();

	}



	void PutNumbers()
	{
		SmartDashboard::PutString("DB/String 0", "Camera Offset: " + std::to_string(bot.GetOffSet()));
		SmartDashboard::PutString("DB/String 1", "Shifter: " + std::to_string(bot.GetShift()));
		SmartDashboard::PutString("DB/String 2", "Gear Catch: " + std::to_string(bot.GetCatch()));
		SmartDashboard::PutString("DB/String 3", "Left Distance: " + std::to_string((int)bot.GetLeftDistance()));
		SmartDashboard::PutString("DB/String 4", "Right Distance: " + std::to_string((int)bot.GetRightDistance()));
		SmartDashboard::PutString("DB/String 5", "Avg Distance: " + std::to_string((int)bot.GetDistance()));
		//SmartDashboard::PutString("DB/String 6", "Turret Rotation: " + std::to_string(balls.turret.GetAvgRotation()));
		SmartDashboard::PutString("DB/String 6", "Left X: " + std::to_string(bot.GetX()));
		SmartDashboard::PutString("DB/String 7", "Climb Current: " + std::to_string(balls.GetAvgClimbCurrent()));
		SmartDashboard::PutString("DB/String 8", "NavX Abs Angle: " + std::to_string(bot.GetAbsoluteAngle()).substr(0,6));
		SmartDashboard::PutString("DB/String 9", "NavX Raw Yaw: " + std::to_string(	bot.GetYaw()).substr(0,6));
		//SmartDashboard::PutString("DB/String 8", "Right: " + std::to_string(right));
	}


	void TestPeriodic() {
		lw->Run();
		LogData("test.log");
	}

	void DisabledInit(){
		StopDataLog();
	}

	// Shortens Strings
	std::string small_str(std::string long_str){
		std::string output = "";
		char c = 0xFF;
		for(unsigned int i = 0; i < long_str.length(); i++){
			if(c & 0xF0 == 0xF0){ //Low Nibble
				c &= 0x00;
				if(long_str.at(i) == '0')
					c = (c & 0xF0) | (_0 & 0xF);
				else if(long_str.at(i) == '1')
					c = (c & 0xF0) | (_1 & 0xF);
				else if(long_str.at(i) == '2')
					c = (c & 0xF0) | (_2 & 0xF);
				else if(long_str.at(i) == '3')
					c = (c & 0xF0) | (_3 & 0xF);
				else if(long_str.at(i) == '4')
					c = (c & 0xF0) | (_4 & 0xF);
				else if(long_str.at(i) == '5')
					c = (c & 0xF0) | (_5 & 0xF);
				else if(long_str.at(i) == '6')
					c = (c & 0xF0) | (_6 & 0xF);
				else if(long_str.at(i) == '7')
					c = (c & 0xF0) | (_7 & 0xF);
				else if(long_str.at(i) == '8')
					c = (c & 0xF0) | (_8 & 0xF);
				else if(long_str.at(i) == '9')
					c = (c & 0xF0) | (_9 & 0xF);
				else if(long_str.at(i) == '.')
					c = (c & 0xF0) | (_dot & 0xF);
				else if(long_str.at(i) == '-')
					c = (c & 0xF0) | (_neg & 0xF);
				else if(long_str.at(i) == ',')
					c = (c & 0xF0) | (_end & 0xF);
			}else{ //High Nibble
				if(long_str.at(i) == '0')
					c = (c & ~0xF0) | ((_0 & 0xF) << 4);
				else if(long_str.at(i) == '1')
					c = (c & ~0xF0) | ((_1 & 0xF) << 4);
				else if(long_str.at(i) == '2')
					c = (c & ~0xF0) | ((_2 & 0xF) << 4);
				else if(long_str.at(i) == '3')
					c = (c & ~0xF0) | ((_3 & 0xF) << 4);
				else if(long_str.at(i) == '4')
					c = (c & ~0xF0) | ((_4 & 0xF) << 4);
				else if(long_str.at(i) == '5')
					c = (c & ~0xF0) | ((_5 & 0xF) << 4);
				else if(long_str.at(i) == '6')
					c = (c & ~0xF0) | ((_6 & 0xF) << 4);
				else if(long_str.at(i) == '7')
					c = (c & ~0xF0) | ((_7 & 0xF) << 4);
				else if(long_str.at(i) == '8')
					c = (c & ~0xF0) | ((_8 & 0xF) << 4);
				else if(long_str.at(i) == '9')
					c = (c & ~0xF0) | ((_9 & 0xF) << 4);
				else if(long_str.at(i) == '.')
					c = (c & ~0xF0) | ((_dot & 0xF) << 4);
				else if(long_str.at(i) == '-')
					c = (c & ~0xF0) | ((_neg & 0xF) << 4);
				else if(long_str.at(i) == ',')
					c = (c & ~0xF0) | ((_end & 0xF) << 4);
			}
			if((i % 2 == 0 && i) || i == long_str.length()-1){ // Every even count will fill a char (not including 0)
				output += c;
				c = 0xFF;
			}

		}

		return output;
	}

	/*std::string decode(std::string str){
		std::string decoded = "";
		for(int i = 0; i < str.length(); i++){
			switch(char mask = str & 0xF0){
							case 0x00:
								decoded += "0";
								break;
							default:
								break;
			}
		}
	}*/

	struct data_packet {
		double angle;
		int renc;
		int lenc;
		double yaw;
		double accx;
		double accy;
		double accz;
		double accposx;
		double accposy;
		double accposz;
	};

	//FILE *tele;
	void LogData(std::string file){
		//std::string accelerometerDistance = std::to_string(bot.GetAccelerometerDistance());
		std::string climber = std::to_string(balls.GetAvgClimbCurrent());
		std::string angle = std::to_string(bot.GetAbsoluteAngle());

		std::string data = std::to_string(Timer::GetMatchTime()) +
				//" D:" + accelerometerDistance.substr(0, accelerometerDistance.length()-3) + // Distance
				"," + angle.substr(0,6) + 		 					 // Angle
				"," + std::to_string((int)bot.GetRightDistance()) +  // Right Encoder Val
				"," + std::to_string((int)bot.GetLeftDistance()) +	 // Left Encoder Val
				"," + std::to_string((int)bot.GetDistance()) +		 // Avg Encoder Val
				"," + std::to_string(bot.GetYaw()).substr(0,6) +	 // Yaw Value
				"," + climber.substr(0, climber.length()-3) +	     // Climb Current
				"," + std::to_string(bot.GetAccelerometerX()) +		 // Accelerometer X
				"," + std::to_string(bot.GetAccelerometerY()) +		 // Accelerometer Y
				"," + std::to_string(bot.GetAccelerometerZ()) +		 // Accelerometer Z
				"," + std::to_string(bot.GetAccelerometerPosX()) +	 // Accelerometer Pos X
				"," + std::to_string(bot.GetAccelerometerPosY()) + 	 // Accelerometer Pos Y
				"," + std::to_string(bot.GetAccelerometerPosZ());	 // Accelerometer Pos Z

		std::string cmd = "echo \"" + data +
								"\" >> /home/lvuser/" + file;

		/*std::string cmd = "T:" + std::to_string(Timer::GetMatchTime()) +
										" D:" + std::to_string(bot.GetAccelerometerDistance()) + // Distance
										" A:" + std::to_string(bot.GetAbsoluteAngle()) + 		 // Angle
										" RE:" + std::to_string((int)bot.GetRightDistance()) + 	 // Right Encoder Val
										" LE:" + std::to_string((int)bot.GetLeftDistance()) +	 // Left Encoder Val
										" AD:" + std::to_string((int)bot.GetDistance()) +		 // Avg Encoder Val
										" RY:" + std::to_string(bot.GetYaw()).substr(0,6) +		 // Yaw Value
										" CD:" + std::to_string(balls.GetAvgClimbCurrent());	 // Climb Current*/

		//fprintf(tele, "%s", cmd.c_str());
		system(cmd.c_str());
	}

	void StartDataLog(std::string file){
		//system("./mount.sh");
		system("cd /home/lvuser/");
		system(("touch " + file).c_str());
		//tele = fopen("/home/lvuser/media/tele.log", "a");

	}

	void StopDataLog(){
		//system("./unmount.sh");
	}

	void DisabledPeriodic(){
		PutNumbers();
	}


	private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "JustinAuto";
	std::string autoSelected;
//	frc::Joystick l_stick {0};
//	frc::Joystick r_stick {1};
//	CANTalon l_motor {3};
//	CANTalon r_motor {5};
//	Encoder  l_drive {0,1};
//	Encoder  r_drive {2,3};
	RoboBase bot {	L_MOTOR_PORT_1,
					L_MOTOR_PORT_2,
					R_MOTOR_PORT_1,
					R_MOTOR_PORT_2,
					L_ENCODER_CH_1,
					L_ENCODER_CH_2,
					R_ENCODER_CH_1,
					R_ENCODER_CH_2,
					SHIFT_PORT,
					GEAR_PORT,
					L_JOYSTICK_PORT,
					R_JOYSTICK_PORT,
					OFFSET_PORT};

	Balls balls {	T_MOTOR_PORT,
		            A_MOTOR_PORT,
					S_MOTOR_PORT,
					I_MOTOR_PORT,
					C_MOTOR_PORT_1,
					C_MOTOR_PORT_2,
					T_ENCODER_CH_1,
					T_ENCODER_CH_2,
					A_ENCODER_CH_1,
					A_ENCODER_CH_2,
					T_JOYSTICK_PORT};
};

START_ROBOT_CLASS(Robot)
