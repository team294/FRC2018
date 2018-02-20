
package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Intake extends Subsystem {

	private final Solenoid intakeOpenPiston = new Solenoid(RobotMap.pneumaticIntakePistonOpen);
	private final Solenoid intakeDeployPiston = new Solenoid(RobotMap.pneumaticIntakePistonDeploy);

	private final TalonSRX intakeMotorLeft = new TalonSRX(RobotMap.intakeMotorLeft);
	private final TalonSRX intakeMotorRight = new TalonSRX(RobotMap.intakeMotorRight);
	private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitchIntake);

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	/**
	 * Opens the intake jaws
	 */
	public void openIntake() {
		intakeOpenPiston.set(true); // true is open
	}

	/**
	 * Closes the intake jaws
	 */
	public void closeIntake() {
		intakeOpenPiston.set(false); // false is close
	}

	/* These private commands have no checks to see if the arm is in a safe state, so they should never be called by commands */
	
	private void deployIntake() {
		intakeDeployPiston.set(true); // true is deploy
	}

	private void retractIntake() {
		intakeDeployPiston.set(false); // false is retract
	}
	
	/**
	 * Deploys the intake only if the robot arm is in a safe range to do so and only if the claw is closed
	 * @return true if the intake deployed successfully, false if not
	 */
	public boolean smartDeployIntake() {
		if (Robot.armMotor.getArmDegrees() < -3000.0 && !Robot.claw.getClawState()) { //TODO: The arm value needs to be changed to the correct range, and we may need to add another check for a lower/upper boundary on the arm
			deployIntake();
			return true;
		}
		return false;
	}
	
	/**
	 * Retracts the intake only if the robot arm is in a safe range to do so and only if the claw is closed
	 * @return true if the intake retracted successfully, false if not
	 */
	public boolean smartRetractIntake() {
		if (Robot.armMotor.getArmDegrees() < -3000.0 && !Robot.claw.getClawState()) { //TODO: The arm value needs to be changed to the correct range, and we may need to add another check for a lower/upper boundary on the arm
			retractIntake();
			return true;
		}
		return false;
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
	
	// Is there any situation where we will need to run the intake motors at different speeds? If not, can we get rid of this unused method?

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
	 * @return true if the jaws closed, false if it did not (i.e. it is still opened)
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
	 * 
	 * @return true = object is breaking the photo beam
	 */
	public boolean getPhotoSwitch() {
		return photoSwitch.get();
	}
	
	/**
	 * Gets the deploy/retract state of the intake
	 * </br><b>Warning: this only checks what state the intake is set to, not what it actually is.</b>
	 * @return true = deployed, false = retracted
	 */
	public boolean getIntakeDeploy() {
		return intakeDeployPiston.get();
	}
	
	/**
	 * Gets the open/close state of the jaws of the intake
	 * @return true = open, false = closed
	 */
	public boolean getIntakeJawState() {
		return intakeOpenPiston.get();
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

}
