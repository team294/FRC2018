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

	private double angleSpeedControl, amountTurn;
	private double prevAngleError = 0, integratedError = 0, derivativeError = 0, angleError = 0;
	private static final double dt = .02;
	private final VelocityChecker velCheck = new VelocityChecker(0.5);
	private final double kPdist = 0.04, // Proportional Term
			kDdist = 0.0036, // Derivative Value
			kIdist = 0; // Integral Term

	public TurnGyro(double amountTurn, Units turnUnits) {
		this.amountTurn = convertAngleToDegrees(amountTurn, turnUnits);
		requires(Robot.driveTrain);
	}

	private double convertAngleToDegrees(double inputAngle, Units turnUnits) {
		switch (turnUnits) {
		case Radians:
			return Math.toDegrees(inputAngle);
		case Rotations:
			return inputAngle * 360.0;
		}
		return inputAngle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		prevAngleError = 0;
		integratedError = 0;
		derivativeError = 0;
		angleError = 0;
		Robot.log.writeLog("Turn Gyro initialized");
		velCheck.clearHistory();
		// Robot.driveTrain.zeroGyroRoataion();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		SmartDashboard.putNumber("Gyro Levels:", Robot.driveTrain.getGyroRotation());
		angleError = amountTurn - Robot.driveTrain.getGyroRotation();
		angleError = angleError < -180 ? angleError + 360 : angleError;
		angleError = angleError > 180 ? angleError - 360 : angleError;
		integratedError += angleError * dt;
		derivativeError = (angleError - prevAngleError) / dt;
		angleSpeedControl = angleError * kPdist + integratedError * kIdist + derivativeError * kDdist;
		// Robot.driveTrain.tankDrive(-percentSpeed, percentSpeed);
		angleSpeedControl = angleSpeedControl < .2 && angleSpeedControl > 0 ? .2 : angleSpeedControl;
		angleSpeedControl = angleSpeedControl > -.2 && angleSpeedControl < 0 ? -.2 : angleSpeedControl;
		Robot.driveTrain.tankDrive(angleSpeedControl, -angleSpeedControl);
		velCheck.addValue(angleError - prevAngleError);
		prevAngleError = angleError;
		SmartDashboard.putNumber("Gyro Turn Dist Err:", angleError);
		SmartDashboard.putNumber("Gyro Turn Perc Speed:", angleSpeedControl);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return velCheck.getAverage() < 1 || Math.abs(angleError) <= 1;
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.log.writeLog(
				"Turn Gyro ended with average of: " + velCheck.getAverage() + ", and a distErr of: " + angleError);
		Robot.driveTrain.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.log.writeLog("Turn Gyro interrupted.");
		end();
	}
}
