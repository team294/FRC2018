
package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intake extends Subsystem {

	// Needs command integration. Should open intake/reset when
	// Robot.inputs.isCubeFullyIn() returns true
	double leftPercent = -50;
	double rightPercent = -50;
	public double actualRightPercent;
	public double actualLeftPercent;
	public double lastLeftPercent = leftPercent;
	public double lastRightPercent = rightPercent;
	private final DoubleSolenoid intakePiston = new DoubleSolenoid(RobotMap.pneumaticIntakePistonIn,
			RobotMap.pneumaticIntakePistonOut);

	private final WPI_TalonSRX intakeMotorLeft = new WPI_TalonSRX(RobotMap.intakeMotorLeft);
	private final WPI_TalonSRX intakeMotorRight = new WPI_TalonSRX(RobotMap.intakeMotorRight);

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void openIntake() {
		intakePiston.set(Value.kForward); // kForward is extend
	}

	public void closeIntake() {
		intakePiston.set(Value.kReverse); // kReverse is retract
	}

	public void setIntakeMotorToPercentPower(double leftPercent, double rightPercent) {
		intakeMotorLeft.set(ControlMode.PercentOutput, leftPercent);
		intakeMotorRight.set(ControlMode.PercentOutput, rightPercent);
		System.out.println("Left Intake motor " + intakeMotorLeft.getDeviceID() + " set to percent " + leftPercent
				+ ", output " + intakeMotorLeft.getMotorOutputVoltage() + " V," + intakeMotorLeft.getOutputCurrent()
				+ " A, Bus at " + intakeMotorLeft.getBusVoltage() + " V");
		System.out.println("Right Intake motor " + intakeMotorRight.getDeviceID() + " set to percent " + rightPercent
				+ ", output " + intakeMotorRight.getMotorOutputVoltage() + " V," + intakeMotorRight.getOutputCurrent()
				+ " A, Bus at " + intakeMotorRight.getBusVoltage() + " V");
		SmartDashboard.putNumber("Left Intake Motor Percent:", leftPercent);
		SmartDashboard.putNumber("Right Intake Motor Percent:", rightPercent);
		if (leftPercent < 0) {
			lastLeftPercent = leftPercent;
			lastRightPercent = rightPercent;
		}
		if (rightPercent < 0) {
			lastLeftPercent = leftPercent;
			lastRightPercent = rightPercent;
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		// sets both intake motors to -50 by default,
		// will run at this speed until ToggleIntake
		// or another command shuts them off or changes speed.
		setIntakeMotorToPercentPower(leftPercent, rightPercent);
	}

	public void closeIntakeWhenObjectPresent() {
		// if object is detected with photoSwitch, close the intake
		if (Robot.inputs.isObjectPresentIntake()) {
			Robot.intake.setIntakeMotorToPercentPower(lastLeftPercent, lastRightPercent);
			Robot.intake.closeIntake();
		}
	}

	public double readRightIntakeMotor() {
		actualRightPercent = SmartDashboard.getNumber("Right Intake Motor Percent:", 0);
		return actualRightPercent;
	}

	public double readLeftIntakeMotor() {
		actualLeftPercent = SmartDashboard.getNumber("Left Intake Motor Percent:", 0);
		return actualLeftPercent;
	}

}
