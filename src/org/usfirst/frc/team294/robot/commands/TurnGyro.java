package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class TurnGyro extends Command {
	public static enum Units {
		Degrees, Radians, Rotations;
	}

	private double percentSpeed, amountTurn;
	private final Units turnUnits;
	private boolean useSDashTemp = false;
	private double prevError = 0, integratedError = 0, derivativeError = 0, distError = 0;
	private final double dt = .02;
	private final double 
			kPdist = 0.04, // Proportional Term
			kDdist = 0.0036, 	// Derivative Value
			kIdist = 0; 	// Integral Term

	public TurnGyro(double amountTurn, Units turnUnits) {// TODO FIX NEGATIVIZATION
		this.amountTurn = amountTurn;
		this.turnUnits = turnUnits;
		requires(Robot.driveTrainSubsystem);
	}
	public TurnGyro() {
		this(0,null);
		useSDashTemp = true;
	}
	private double getDegreesTurn() {
		if(useSDashTemp) {
			return SmartDashboard.getNumber("Gyro Turn Angle Input", 180);
		}
		switch (turnUnits) {
		case Radians:
			return Math.toDegrees(amountTurn);
		case Rotations:
			return amountTurn * 360.0;
		}
		return amountTurn;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		prevError = 0;
		integratedError = 0;
		derivativeError = 0;
		distError = 0;
		//Robot.driveTrainSubsystem.zeroGyroRoataion();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		SmartDashboard.putNumber("Gyro Levels:", Robot.driveTrainSubsystem.getGyroRotation());
		distError = getDegreesTurn() - Robot.driveTrainSubsystem.getGyroRotation();
		distError = distError < -180 ? distError + 360 : distError;
		distError = distError > 180 ? distError - 360 : distError;
		integratedError += distError * dt;
		derivativeError = (distError - prevError) / dt;
		percentSpeed = distError * kPdist + integratedError * kIdist + derivativeError * kDdist;
		//Robot.driveTrainSubsystem.tankDrive(-percentSpeed, percentSpeed);
		percentSpeed = percentSpeed < .2 && percentSpeed > 0 ? .2 : percentSpeed;
		percentSpeed = percentSpeed > -.2 && percentSpeed < 0 ? -.2 : percentSpeed;
		Robot.driveTrainSubsystem.tankDrive(percentSpeed, -percentSpeed);
		prevError = distError;
		SmartDashboard.putNumber("Gyro Turn Dist Err:", distError);
		SmartDashboard.putNumber("Gyro Turn Perc Speed:", percentSpeed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(distError) <= 1;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrainSubsystem.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}