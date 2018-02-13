
package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intake extends Subsystem {

	private final DoubleSolenoid intakePiston = new DoubleSolenoid(RobotMap.pneumaticIntakePistonIn,
			RobotMap.pneumaticIntakePistonOut);

	private final TalonSRX intakeMotorLeft = new TalonSRX(RobotMap.intakeMotorLeft);
	private final TalonSRX intakeMotorRight = new TalonSRX(RobotMap.intakeMotorRight);
	private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitchIntake);

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void openIntake() {
		intakePiston.set(Value.kForward); // kForward is extend
	}

	public void closeIntake() {
		intakePiston.set(Value.kReverse); // kReverse is retract
	}

//	public void setIntakeMotorToPercentPower(double leftPercent, double rightPercent) {
//		intakeMotorLeft.set(ControlMode.PercentOutput, leftPercent);
//		intakeMotorRight.set(ControlMode.PercentOutput, rightPercent);
//		System.out.println("Left Intake motor " + intakeMotorLeft.getDeviceID() + " set to percent " + leftPercent
//				+ ", output " + intakeMotorLeft.getMotorOutputVoltage() + " V," + intakeMotorLeft.getOutputCurrent()
//				+ " A, Bus at " + intakeMotorLeft.getBusVoltage() + " V");
//		System.out.println("Right Intake motor " + intakeMotorRight.getDeviceID() + " set to percent " + rightPercent
//				+ ", output " + intakeMotorRight.getMotorOutputVoltage() + " V," + intakeMotorRight.getOutputCurrent()
//				+ " A, Bus at " + intakeMotorRight.getBusVoltage() + " V");
//		SmartDashboard.putNumber("Left Intake Motor Percent:", leftPercent);
//		SmartDashboard.putNumber("Right Intake Motor Percent:", rightPercent);
//	}
	
	
	/**
	 * sets the intake motors to a percentage
	 * @param percent
	 */
	public void setIntakeMotorPercent(double percent) {
		intakeMotorLeft.set(ControlMode.PercentOutput, percent);
		intakeMotorRight.set(ControlMode.PercentOutput, percent);
		System.out.println("Left Intake motor " + intakeMotorLeft.getDeviceID() + " set to percent " + percent
				+ ", output " + intakeMotorLeft.getMotorOutputVoltage() + " V," + intakeMotorLeft.getOutputCurrent()
				+ " A, Bus at " + intakeMotorLeft.getBusVoltage() + " V");
		System.out.println("Right Intake motor " + intakeMotorRight.getDeviceID() + " set to percent " + percent
				+ ", output " + intakeMotorRight.getMotorOutputVoltage() + " V," + intakeMotorRight.getOutputCurrent()
				+ " A, Bus at " + intakeMotorRight.getBusVoltage() + " V");
		SmartDashboard.putNumber("Left Intake Motor Percent:", percent);
		SmartDashboard.putNumber("Right Intake Motor Percent:", percent);
	}

	/**
	 * closes the intake jaws if the photo switch is triggered
	 * @return true if closed, false if opened
	 */
	public boolean smartCloseIntake() {
		// if object is detected with photoSwitch, close the intake
		if (photoSwitch.get()) {
			closeIntake();
			return true;
		}
		return false;
	}
	
	/**
	 * opens the intake jaws and sets the motors to reverse
	 */
	public void outtake() { 
		setIntakeMotorPercent(RobotMap.intakePercentOut);
		openIntake();
	}
	
	/**
	 * stops the intake motors
	 */
	public void stop() {
		setIntakeMotorPercent(0.0); 
	}
	
	/**
	 * Reads the value of the photo switch
	 * @return true = object is breaking the photo beam
	 */
	public boolean getPhotoSwitch() {
		return photoSwitch.get();
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		// sets both intake motors to -50 by default,
		// will run at this speed until ToggleIntake
		// or another command shuts them off or changes speed.
	}

}
