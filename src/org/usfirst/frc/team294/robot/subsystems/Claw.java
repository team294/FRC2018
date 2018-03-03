package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.ArmMotorSetToZero;
import org.usfirst.frc.team294.robot.commands.ClawMotorSetToZero;
import org.usfirst.frc.team294.robot.triggers.MotorCurrentTrigger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Claw extends Subsystem {

	private final DigitalInput bumpSwitch = new DigitalInput(RobotMap.bumpSwitchClaw);
	private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitchClaw);
	private final Solenoid clawPiston = new Solenoid(RobotMap.pneumaticClawPistonOut);

	private final TalonSRX clawMotorLeft = new TalonSRX(RobotMap.clawMotorLeft);
	private final TalonSRX clawMotorRight = new TalonSRX(RobotMap.clawMotorRight);
	
	public final MotorCurrentTrigger clawMotorLeftCurrentTrigger =  new MotorCurrentTrigger(clawMotorLeft, 20, 2);
	public final MotorCurrentTrigger clawMotorRightCurrentTrigger =  new MotorCurrentTrigger(clawMotorRight, 20, 2);
	
	public Claw() {
		// Configure talons
		clawMotorLeft.set(ControlMode.PercentOutput, 0);
		clawMotorLeft.setNeutralMode(NeutralMode.Coast);
		clawMotorLeft.enableVoltageCompensation(true);
		clawMotorLeft.configVoltageCompSaturation(11.0, 0);
		clawMotorLeft.configOpenloopRamp(0.2, 0);

		clawMotorRight.set(ControlMode.PercentOutput, 0);
		clawMotorRight.setInverted(true);
		clawMotorRight.setNeutralMode(NeutralMode.Coast);
		clawMotorRight.enableVoltageCompensation(true);
		clawMotorRight.configVoltageCompSaturation(11.0, 0);
		clawMotorRight.configOpenloopRamp(0.2, 0);
	}
	
	/**
	 * Adds current protection to the claw motor. If the claw motor trips this, the claw will stop
	 */
	public void clawMotorsCurrentProtection(){
		clawMotorLeftCurrentTrigger.whenActive(new ClawMotorSetToZero());
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void openClaw() {										// this logic is funky  Clean up
		/* double currentAngle = Robot.armMotor.getArmDegrees();
		if (currentAngle <= RobotMap.angleClawCloseHigh && currentAngle >= RobotMap.angleClawCloseLow) {
		}
		else { */ // no longer need keep-out zone
		clawPiston.set(true); // true is extend
		//}
	}

	public void closeClaw() {
		clawPiston.set(false); // false is retract
	}

	public void setClawMotorToPercentPower(double leftPercent, double rightPercent) {
		clawMotorLeft.set(ControlMode.PercentOutput, leftPercent);
		clawMotorRight.set(ControlMode.PercentOutput, rightPercent);
		System.out.println("Left Claw motor " + clawMotorLeft.getDeviceID() + " set to percent " + leftPercent + ", output "
				+ clawMotorLeft.getMotorOutputVoltage() + " V," + clawMotorLeft.getOutputCurrent() + " A, Bus at "
				+ clawMotorLeft.getBusVoltage() + " V");
		System.out.println("Right Claw motor " + clawMotorRight.getDeviceID() + " set to percent " + rightPercent
				+ ", output " + clawMotorRight.getMotorOutputVoltage() + " V," + clawMotorRight.getOutputCurrent()
				+ " A, Bus at " + clawMotorRight.getBusVoltage() + " V");
		SmartDashboard.putNumber("Left Claw Motor Percent:", leftPercent);
		SmartDashboard.putNumber("Right Claw Motor Percent:", rightPercent);
		
	}
	
	public void setClawMotorPercent(double percent) {
		clawMotorLeft.set(ControlMode.PercentOutput, percent);
		clawMotorRight.set(ControlMode.PercentOutput, percent);
		System.out.println("Left Claw motor " + clawMotorLeft.getDeviceID() + " set to percent " + percent + ", output "
				+ clawMotorLeft.getMotorOutputVoltage() + " V," + clawMotorLeft.getOutputCurrent() + " A, Bus at "
				+ clawMotorLeft.getBusVoltage() + " V");
		System.out.println("Right Claw motor " + clawMotorRight.getDeviceID() + " set to percent " + percent
				+ ", output " + clawMotorRight.getMotorOutputVoltage() + " V," + clawMotorRight.getOutputCurrent()
				+ " A, Bus at " + clawMotorRight.getBusVoltage() + " V");
		SmartDashboard.putNumber("Left Claw Motor Percent:", percent);
		SmartDashboard.putNumber("Right Claw Motor Percent:", percent);
		
	}
	
	/**
	 * Reads the value of the bump switch
	 * @return true = object is pressing bump switch
	 */
	public boolean getBumpSwitch() {
		return bumpSwitch.get();
	}
	
	/**
	 * Reads the value of the photo switch
	 * @return true = object is breaking the photo beam
	 */
	public boolean getPhotoSwitch() {
		return photoSwitch.get();
	}
	
	/**
	 * closes the intake jaws if the photo switch is triggered
	 * @return true if closed, false if opened
	 */
	public boolean clawCloseIfPhotoSwitch() {
		// if object is detected with photoSwitch, close the intake
		if (photoSwitch.get()) {
			closeClaw();
			return true;
		} else 
		return false;
	}
	
	/**
	 * stops the claw motors
	 */
	public void stop() {
		setClawMotorPercent(0.0);
	}

	/* public double getClawMotorPower() {
		double voltage = (clawMotorLeft.getMotorOutputPercent() + clawMotorRight.getMotorOutputPercent()) / 2;
		SmartDashboard.putNumber("Arm Motor Output Voltage", voltage);
		return voltage;
	} */

	public void periodic() {
		SmartDashboard.putBoolean("PhotoSwitch Triggered", getPhotoSwitch());
		SmartDashboard.putBoolean("Cube Present", getBumpSwitch());
		
		SmartDashboard.putBoolean("Arm Photo", photoSwitch.get());
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
