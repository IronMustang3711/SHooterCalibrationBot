#include <iostream>
#include <memory>
#include <string>

#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <RobotDrive.h>
#include <Timer.h>
#include <CANTalon.h>
#include <Talon.h>
#include <PowerDistributionPanel.h>

/**
 While throttling the Talon in the positive direction, make sure the sensor speed is also positive.
Additionally note the approximate sensor speed while the Talon is driven.
 getSpeed() will return the speed in RPM since the requirements in Section 17.2.1 are met.

 The SmartDashboard values can also be read in the roboRIO web-based configuration under Self-Tests.

 17.2.1 native units:
 Quadrature Encoder:
 native units per rotation(resolution)= 4 * cycles per revolution,
 reqs. for unit scaling=ConfigEncoderCodesPerRev()

rpm to native units:

X_Rpm * 1/600 = revs per 100ms
revs_per_100ms * native_units_per_rotation =  native units per 100ms

1023 = max out

F-gain = (100% X 1023) / (native units per 100ms)

then update F-gain

Remember “err” is in native units per 100ms. So an error of 900 units per 100ms equals an error of 131RPM since each rotation is 4096 units.

Next we will add in P-gain so that the closed-loop can react to error.
 Suppose given our worst error so far (900 native units per 100ms), we want to respond with another 10% of throttle.
 Then our starting p-gain would be....
(10% X 1023) / (900) = 0.113333
Now let’s check our math, if the Talon SRX sees an error of 900 the P-term will be
900 X 0.113333 = 102 (which is about 10% of 1023) P-gain = 0.113333
Apply the P -gain programmatically using your preferred method.
Now retest to see how well the closed-loop responds to varying loads.
Double the P -gain until the system oscillates (too much) or until the system responds adequately.
If the mechanism is moving to swiftly, you can add D-gain to smooth the motion. Start with 10x the p-gain.
If the mechanism is not quite reaching the final target position (and P -gain cannot be increased further without hurting overall performance)
 begin adding I-gain. Start with 1/100th of the P-gain.

 */
class Robot: public frc::SampleRobot {
	frc::Joystick stick { 0 };
	CANTalon shooterController { 1 /*update rate in ms*/};
	frc::Talon loaderController { 5 };
	frc::PowerDistributionPanel pdp{};


public:
	Robot() {
		//Note SmartDashboard is not initialized here, wait until RobotInit to make SmartDashboard calls

	}

	void RobotInit() {
	      shooterController.SetFeedbackDevice(CANTalon::QuadEncoder);
	      shooterController.ConfigEncoderCodesPerRev(20);
	      shooterController.SetSensorDirection(false);
			shooterController.SetPosition(0);
			shooterController.SetControlMode(CANSpeedController::kSpeed);

//Nominal Closed-Loop Output: Promotes the minimal or weakest motor-output during closed-loop.
	      shooterController.ConfigNominalOutputVoltage(+0., -2.0);
	      shooterController.ConfigPeakOutputVoltage(-2.0, -12.0);
			/* set the allowable closed-loop error,
			 * Closed-Loop output will be neutral within this range.
			 * See Table in Section 17.2.1 for native units per rotation.
			 */
	      shooterController.SetAllowableClosedLoopErr(0); /* always servo */
	     // shooterController.SelectProfileSlot(1); //CONFIGURE PID SLOT 1 in web dashboard
//	      shooterController.SetF(1.8);
//	      shooterController.SetP(0.38);
//	      shooterController.SetI(0.0);
//	      shooterController.SetD(0.0);
//	      shooterController.SetCloseLoopRampRate(0.0);


	}
	void shooterUpdate() {
		float sliderValue = (-stick.GetRawAxis(3)+1)*0.5;   // this was reversed
		SmartDashboard::PutNumber("slider", sliderValue);
		double motorOutput = shooterController.GetOutputVoltage()
				/ shooterController.GetBusVoltage();

		SmartDashboard::PutNumber("TALON: motor output", motorOutput);
		SmartDashboard::PutNumber("TALON: position",
				shooterController.GetPosition());
		SmartDashboard::PutNumber("TALON: speed", shooterController.GetSpeed());

	//	if (stick.GetRawButton(2)) {
	//		shooterController.SetControlMode(CANSpeedController::kSpeed);
			double target = -sliderValue * 4500.0; //RPM
			shooterController.Set(target);

			SmartDashboard::PutNumber("TALON: target", target);

//		} else {
//			shooterController.SetControlMode(CANSpeedController::kPercentVbus);
//			shooterController.Set(sliderValue);
//		}
		SmartDashboard::PutNumber("TALON: closed loop error",
				shooterController.GetClosedLoopError());


		SmartDashboard::PutNumber("encoder value",
				shooterController.GetEncPosition());
		SmartDashboard::PutNumber("encoder velocity",
				shooterController.GetEncVel());
		SmartDashboard::PutNumber("encoder position",
				shooterController.GetPosition());
		SmartDashboard::PutNumber("current",pdp.GetCurrent(12));
	}

	/*
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl() override {

		while (IsOperatorControl() && IsEnabled()) {
			if (stick.GetRawButton(1)) {
				loaderController.Set(1.0);
			} else {
				loaderController.Set(0.0);
			}
			shooterUpdate();

			frc::Wait(0.005);
		}
	}

};

START_ROBOT_CLASS(Robot)
