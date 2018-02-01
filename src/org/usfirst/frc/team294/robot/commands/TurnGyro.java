package org.usfirst.frc.team294.robot.commands;

import java.util.Arrays;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.utilities.VelocityChecker;

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
	private double prevError = 0, integratedError = 0, derivativeError = 0, distError = 0;
	private static final double dt = .02;
	private final VelocityChecker velCheck = new VelocityChecker(1);
	private final double 
			kPdist = 0.04, // Proportional Term
			kDdist = 0.0036, 	// Derivative Value
			kIdist = 0; 	// Integral Term

	public TurnGyro(double amountTurn, Units turnUnits) {// TODO FIX NEGATIVIZATION
		this.amountTurn = amountTurn;
		this.turnUnits = turnUnits;
		requires(Robot.driveTrain);
	}
	private double getDegreesTurn() {
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
		Robot.log.writeLog("Turn Gyro initialized");
		//Robot.driveTrain.zeroGyroRoataion();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		SmartDashboard.putNumber("Gyro Levels:", Robot.driveTrain.getGyroRotation());
		distError = getDegreesTurn() - Robot.driveTrain.getGyroRotation();
		distError = distError < -180 ? distError + 360 : distError;
		distError = distError > 180 ? distError - 360 : distError;
		integratedError += distError * dt;
		derivativeError = (distError - prevError) / dt;
		percentSpeed = distError * kPdist + integratedError * kIdist + derivativeError * kDdist;
		//Robot.driveTrain.tankDrive(-percentSpeed, percentSpeed);
		percentSpeed = percentSpeed < .2 && percentSpeed > 0 ? .2 : percentSpeed;
		percentSpeed = percentSpeed > -.2 && percentSpeed < 0 ? -.2 : percentSpeed;
		Robot.driveTrain.tankDrive(percentSpeed, -percentSpeed);
		prevError = distError;
		velCheck.addValue(distError-prevError);
		SmartDashboard.putNumber("Gyro Turn Dist Err:", distError);
		SmartDashboard.putNumber("Gyro Turn Perc Speed:", percentSpeed);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return velCheck.getAverage()<1|Math.abs(distError) <= 1 ;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.log.writeLog("Turn Gyro ended with average of: "+velCheck.getAverage()+", and a distErr of: "+distError);
		Robot.driveTrain.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.log.writeLog("Turn Gyro interrupted.");
		end();
	}
}
