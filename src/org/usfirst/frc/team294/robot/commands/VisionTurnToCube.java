package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;


/**
 *
 */
public class VisionTurnToCube extends Command {
	boolean done;

	public VisionTurnToCube() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		double RPiX;
		RPiX = Robot.visionData.RPiX;
		if (Robot.visionData.RPiX <= 330 || Robot.visionData.RPiX >= 310) {
			done = true;
		}

	}

	protected void execute() {
	double currentDriveTrainAngle = Robot.driveTrain.getGyroRotation();

		if(Robot.visionData.RPiX < 310) {
			new TurnGyro(currentDriveTrainAngle +1, TurnGyro.Units.Degrees); 
		}
		else if (Robot.visionData.RPiX > 330) {
			new TurnGyro(currentDriveTrainAngle -1, TurnGyro.Units.Degrees); 
		}
		else {
			done = true; 
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
