
package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.ClawMotorSetToZero;
import org.usfirst.frc.team294.robot.commands.IntakeMotorSetToZero;
import org.usfirst.frc.team294.robot.triggers.MotorCurrentTrigger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intake extends Subsystem {

	private final Solenoid intakeOpenPiston = new Solenoid(RobotMap.pneumaticIntakePistonOpen);
	private final DoubleSolenoid intakeDeployPiston = new DoubleSolenoid(RobotMap.pneumaticIntakePistonDeploy, RobotMap.pneumaticIntakePistonStow);  // Forward = deployed

	private final TalonSRX intakeMotorLeft = new TalonSRX(RobotMap.intakeMotorLeft);
	private final TalonSRX intakeMotorRight = new TalonSRX(RobotMap.intakeMotorRight);
	private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitchIntake);
	public static boolean cubeInIntake;

	
	public final MotorCurrentTrigger intakeMotorLeftCurrentTrigger =  new MotorCurrentTrigger(intakeMotorLeft, 8, 4);
	public final MotorCurrentTrigger intakeMotorRightCurrentTrigger =  new MotorCurrentTrigger(intakeMotorRight, 8, 4);

	public Intake() {
	intakeMotorLeft.set(ControlMode.PercentOutput, 0);
	intakeMotorLeft.setNeutralMode(NeutralMode.Brake);
	intakeMotorLeft.enableVoltageCompensation(true);
	intakeMotorLeft.configVoltageCompSaturation(11.0, 0);
	intakeMotorLeft.setInverted(true);

	intakeMotorRight.set(ControlMode.PercentOutput, 0);
	intakeMotorRight.setNeutralMode(NeutralMode.Brake);
	intakeMotorRight.enableVoltageCompensation(true);
	intakeMotorRight.configVoltageCompSaturation(11.0, 0);
	intakeMotorRight.setInverted(false);
	cubeInIntake = false;
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	/**
	 * Adds current protection to the intake motor. If the intake motor trips this, the intake will stop
	 */
	public void intakeMotorsCurrentProtection(){
		intakeMotorLeftCurrentTrigger.whenActive(new IntakeMotorSetToZero());
		intakeMotorRightCurrentTrigger.whenActive(new IntakeMotorSetToZero());
	}
	
	/**
	 * Returns the state of the intake grabbers.
	 * @return true = open, false = closed
	 */
	public boolean isIntakeOpen() {
		return intakeOpenPiston.get();
	}

	/**
	 * Deploys or retracts the intake based on parameter
	 * @param deployed true = deployed, false = retracted
	 */
	public void setIntakeDeploy(boolean deployed) {
		if (!deployed) {
			if (Robot.armMotor.getArmDegrees() > (RobotMap.armIntakeClearanceAng + 3)) {
				intakeDeployPiston.set(DoubleSolenoid.Value.kReverse);
				
			} else if (Robot.armMotor.getArmDegrees() < (RobotMap.minAngle + 3)) {
				intakeDeployPiston.set(DoubleSolenoid.Value.kReverse);
			} else {
				intakeDeployPiston.set(DoubleSolenoid.Value.kForward);
			}
		} else {
			intakeDeployPiston.set(DoubleSolenoid.Value.kForward);
		}
		stop();
	}
	
	/**
	 * Open or close the jaws on the intake
	 * @param open true = open, false = close
	 */
	public void setIntakeOpen(boolean open) {
		intakeOpenPiston.set(open);
	}
	
	public void updateCubeStatus() {
		cubeInIntake = !cubeInIntake;
	}
	public boolean isCubeInIntake() {
		return cubeInIntake;
	}
	
	// public void setIntakeMotorToPercentPower(double leftPercent, double
	// rightPercent) {
	// intakeMotorLeft.set(ControlMode.PercentOutput, leftPercent);
	// intakeMotorRight.set(ControlMode.PercentOutput, rightPercent);
	// System.out.println("Left Intake motor " + intakeMotorLeft.getDeviceID() + "
	// set to percent " + leftPercent
	// + ", output " + intakeMotorLeft.getMotorOutputVoltage() + " V," +
	// intakeMotorLeft.getOutputCurrent()
	// + " A, Bus at " + intakeMotorLeft.getBusVoltage() + " V");
	// System.out.println("Right Intake motor " + intakeMotorRight.getDeviceID() + "
	// set to percent " + rightPercent
	// + ", output " + intakeMotorRight.getMotorOutputVoltage() + " V," +
	// intakeMotorRight.getOutputCurrent()
	// + " A, Bus at " + intakeMotorRight.getBusVoltage() + " V");
	// SmartDashboard.putNumber("Left Intake Motor Percent:", leftPercent);
	// SmartDashboard.putNumber("Right Intake Motor Percent:", rightPercent);
	// }

	/**
	 * sets the intake motors to a percentage
	 * 
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
	 * 
	 * @return true once claw is closed, false otherwise
	 */
	public boolean smartCloseIntake() {
		// if object is detected with photoSwitch, close the intake
		if (photoSwitch.get()) {
			setIntakeOpen(false);
			return true;
		} else
		return false;
	}

	/**
	 * opens the intake jaws and sets the motors to reverse
	 */
	public void outtake() {
		setIntakeMotorPercent(RobotMap.intakePercentOut);
	}

	/**
	 * stops the intake motors
	 */
	public void stop() {
		setIntakeMotorPercent(0.0);
	}

	/**
	 * Reads the value of the photo switch
	 * 
	 * @return true = object is breaking the photo beam
	 */
	public boolean getPhotoSwitch() {
		return photoSwitch.get();
	}
	
	/**
	 * Gets the state of the intake
	 * @return true = deployed, false = retracted or unknown state
	 */
	public boolean isIntakeDeployed() {
		return intakeDeployPiston.get() == DoubleSolenoid.Value.kForward;
	}

	public void periodic() {
		SmartDashboard.putBoolean("Object Present (Intake): ", getPhotoSwitch());
		SmartDashboard.putBoolean("Intake Photo", photoSwitch.get());
		SmartDashboard.putNumber("Intake Left Motor voltage", intakeMotorLeft.getMotorOutputVoltage());
		SmartDashboard.putNumber("Intake Right Motor voltage", intakeMotorRight.getMotorOutputVoltage());
		SmartDashboard.putNumber("Intake Left Motor current", intakeMotorLeft.getOutputCurrent());
		SmartDashboard.putNumber("Intake Right Motor current", intakeMotorRight.getOutputCurrent());
		SmartDashboard.putBoolean("Intake deployed", intakeDeployPiston.get() ==DoubleSolenoid.Value.kForward);
	}
	

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		// sets both intake motors to -50 by default,
		// will run at this speed until ToggleIntake
		// or another command shuts them off or changes speed.
	}

}
