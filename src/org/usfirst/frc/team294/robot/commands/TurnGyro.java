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
	private final VelocityChecker velCheck = new VelocityChecker(.8);
	private final double kPdist = 0.045, // Proportional Term
			kDdist = 0.0038, // Derivative Value
			kIdist = 0; // Integral Term
	private boolean useVisionForAngle;

	/**
	 * Turns to the angle specified by amountTurn
	 * 
	 * @param amountTurn
	 *            Amount to turn, in turnUnits (+ = right, - = left)
	 * @param turnUnits
	 *            Units enum (Degrees, Radians, or Rotations)
	 */
	public TurnGyro(double amountTurn, Units turnUnits) {// does not use vision
		this.amountTurn = convertAngleToDegrees(amountTurn, turnUnits);
		requires(Robot.driveTrain);
		useVisionForAngle = false;
	}

	/**
	 * Turns to face the nearest cube in front of the robot. If no cube is seen by
	 * the camera, then does not turn.
	 */
	public TurnGyro() {// uses vision
		useVisionForAngle = true;
		requires(Robot.driveTrain);
	}

	// * if not found, then amountTurn = (getCurrentAngle)
	// * if found, convert X location (pixels) to degrees camera vision = 68.5
	// 9.3 pixels = 1 degree
	// TurnGyro( degreesFromVision, Units.Degrees);

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
		if (useVisionForAngle) {
			double RPiX; // the x coordinate in pixels for the center of the conuntour
			double distanceFromCenter;// tells the distance from the center of the screen
			double amountTurnUsingPixels;// tells how much to turn the gyro based on Vision pixels
			Robot.visionData.readCameraData();
			RPiX = Robot.visionData.RPiX; // * Get X location of cube.
			if (RPiX == -1 || (RPiX >= 310 && RPiX <= 330)) {
				amountTurn = Robot.driveTrain.getGyroRotation();
			} else {
				distanceFromCenter = RPiX - 320;
				amountTurnUsingPixels = Math.atan(distanceFromCenter / (470*1.5) ) * (180 / Math.PI);   // Nom dist was 470
				amountTurn = Robot.driveTrain.getGyroRotation() + amountTurnUsingPixels;
			}
		}
		prevAngleError = 0;
		integratedError = 0;
		derivativeError = 0;
		angleError = 0;
		Robot.log.writeLog("Turn Gyro initialized");
		velCheck.clearHistory();
		// Robot.driveTrain.zeroGyroRoataion();

		Robot.log.writeLogEcho("Amount to turn: " + amountTurn);

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
		angleSpeedControl = angleSpeedControl < .4 && angleSpeedControl > 0 ? .4 : angleSpeedControl;
		angleSpeedControl = angleSpeedControl > -.4 && angleSpeedControl < 0 ? -.4 : angleSpeedControl;
		Robot.driveTrain.tankDrive(angleSpeedControl, -angleSpeedControl);
		velCheck.addValue(angleError - prevAngleError);
		prevAngleError = angleError;
		SmartDashboard.putNumber("Gyro Turn Dist Err:", angleError);
		SmartDashboard.putNumber("Gyro Turn Perc Speed:", angleSpeedControl);
		SmartDashboard.putNumber("Degrees to turn Robot: ", amountTurn);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// return velCheck.getAverage() < 1 || Math.abs(angleError) <= 1;
		return velCheck.getAverage() < 1;
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
