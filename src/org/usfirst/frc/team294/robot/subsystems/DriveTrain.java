package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A Subsystem to control the Drive Train
 */
public class DriveTrain extends Subsystem {
	private static final TalonSRX rightMotor1 = new TalonSRX(RobotMap.rightMotor1); //Maps Motors
	private static final TalonSRX rightMotor2 = new TalonSRX(RobotMap.rightMotor2);
	private static final TalonSRX rightMotor3 = new TalonSRX(RobotMap.rightMotor3);
	private static final TalonSRX leftMotor1 = new TalonSRX(RobotMap.leftMotor1);   //Built for 4-Sim Drive Train
	private static final TalonSRX leftMotor2 = new TalonSRX(RobotMap.leftMotor2);
	private static final TalonSRX leftMotor3 = new TalonSRX(RobotMap.leftMotor3);
	public DriveTrain(){//initialize Followers
		leftMotor2.set(ControlMode.Follower, RobotMap.leftMotor1);
		leftMotor3.set(ControlMode.Follower, RobotMap.leftMotor1);
		rightMotor2.set(ControlMode.Follower, RobotMap.rightMotor1);
		rightMotor3.set(ControlMode.Follower, RobotMap.rightMotor1);
	}
	public void setLeftMotors(double power) {//sets power of the left motors
		leftMotor1.set(ControlMode.PercentOutput, -power);//motors are backwards
	}
	public void setRightMotors(double power) {//sets power to the right motors
		rightMotor1.set(ControlMode.PercentOutput, power);
	}
	public void stopAllMotors() {//stops all motorss
		setLeftMotors(0);
		setRightMotors(0);
	}
	public void initDefaultCommand() {
	}
}
