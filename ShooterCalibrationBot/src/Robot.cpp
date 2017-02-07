#include <iostream>
#include <memory>
#include <cmath>

#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SmartDashboard.h>
#include <RobotDrive.h>
#include <CANTalon.h>
#include <Timer.h>

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
	frc::Joystick stick;
	CANTalon shooterController;
	//target in [-4500,0];
	double target = 0;
	double motorOutput = 0;
	double speed = 0;
	int error = 0;
	double timestamp = 0;

public:
	Robot(): stick(0),shooterController(1,5)
	,target(0),motorOutput(0),speed(0),error(0),timestamp(0){
	}

	/**
	 * [0,0.1) -> [0,3000)
	 * [0.1,1.0]->[3000,4500]
	 */
//TODO update numbers for new max rpm, test to see if this provides better control than log scaling.
	double remapSliderValue(double input) {
		return input < 0.1 ? input * 30000.0 : input * 1666.7 + 2833.3;
	}

	void telemetryUpdate() {
		motorOutput = shooterController.GetOutputVoltage()
				/ shooterController.GetBusVoltage();
		speed = shooterController.GetSpeed();
		error = shooterController.GetClosedLoopError();
		timestamp = frc::Timer::GetFPGATimestamp();
	}

	void smartDashboardUpdate() {
		SmartDashboard::PutNumber("TALON: motor output", motorOutput);
		SmartDashboard::PutNumber("TALON: speed", speed);
		SmartDashboard::PutNumber("TALON: target", target);
		SmartDashboard::PutNumber("TALON: closed loop error", error);
	}

	void RobotInit() {
		shooterController.SetFeedbackDevice(CANTalon::QuadEncoder);
		shooterController.ConfigEncoderCodesPerRev(20);
		shooterController.SetSensorDirection(false);
		shooterController.SetPosition(0);
		shooterController.SetControlMode(CANSpeedController::kSpeed);

//Nominal Closed-Loop Output: Promotes the minimal or weakest motor-output during closed-loop.
		shooterController.ConfigNominalOutputVoltage(+0., -2.0);
		shooterController.ConfigPeakOutputVoltage(-2.0, -15.0);
		/* set the allowable closed-loop error,
		 * Closed-Loop output will be neutral within this range.
		 * See Table in Section 17.2.1 for native units per rotation.
		 */
		shooterController.SetAllowableClosedLoopErr(0); /* always servo */
		shooterController.SetF(1.51);
		shooterController.SetP(0.98);
		shooterController.SetI(0.05);
		shooterController.SetD(0.1);
		shooterController.SetCloseLoopRampRate(0.0);
		shooterController.SetIzone(60);


	}
	void shooterUpdate() {
		//sliderValue in [0,1]
		double sliderValue = (-stick.GetRawAxis(3) + 1) * 0.5;
		//target in [-5227,0];
		target = -5227.0 * log10(9.0 * sliderValue + 1.0);

		shooterController.Set(target);

	}

	void OperatorControl() override {

		while (IsOperatorControl() && IsEnabled()) {

			shooterUpdate();
			telemetryUpdate();
			smartDashboardUpdate();

			frc::Wait(0.005);
		}
	}

	void Test() {
		LiveWindow::GetInstance()->AddActuator("ROBOT", "SHOOTER",
				shooterController);
	}

};

START_ROBOT_CLASS(Robot)
