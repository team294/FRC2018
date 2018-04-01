package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team294.robot.commands.*;
import edu.wpi.first.wpilibj.DriverStation;

/**
 *
 */
public class Climb extends Subsystem {

	// We are using the power plug for climbMotor1 to run the intake LEDs, so
	// commenting out the talon until we get power to it.
	private final TalonSRX climbMotor1 = new TalonSRX(RobotMap.climbMotor1);
	private final TalonSRX climbMotor2 = new TalonSRX(RobotMap.climbMotor2);
	private final Solenoid climbPiston = new Solenoid(RobotMap.pneumaticClimbPistonRetract);

	public Climb() {
		// Configure talons
		climbMotor2.set(ControlMode.PercentOutput, 0);
		climbMotor2.setNeutralMode(NeutralMode.Brake);
		climbMotor2.enableVoltageCompensation(true);
		climbMotor2.configVoltageCompSaturation(11.0, 0);
		climbMotor1.set(ControlMode.PercentOutput, 0);
		climbMotor1.setNeutralMode(NeutralMode.Brake);
		climbMotor1.enableVoltageCompensation(true);
		climbMotor1.configVoltageCompSaturation(11.0, 0);
		climbMotor1.set(ControlMode.Follower, RobotMap.climbMotor2);
	}


	// Deploys Climb Piston, but only in the last 30 seconds of the match
	public void deployClimbPiston(boolean state) {
		//Commented out for practice because there is no match time
		//if (DriverStation.getInstance().getMatchTime() <= 30) {
		climbPiston.set(state);
		}
	//}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	/**
	 * Sets the climb motor based on a percent power parameter
	 * 
	 * @param percentPower
	 *            -1 to +1
	 */
	public void setClimbMotors(double percentPower) {
		climbMotor2.set(ControlMode.PercentOutput, percentPower);
	}
	
}
