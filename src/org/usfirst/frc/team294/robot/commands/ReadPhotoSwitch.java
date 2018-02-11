package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.subsystems.Inputs;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ReadPhotoSwitch extends Command {

	public ReadPhotoSwitch() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.inputs.isObjectPresentClaw();
		Robot.inputs.isObjectPresentIntake();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.inputs.isObjectPresentClaw();
		Robot.inputs.isObjectPresentIntake();
		SmartDashboard.putBoolean("Object Present (Claw): ", Robot.inputs.isObjectPresentClaw());
		SmartDashboard.putBoolean("Object Present (Intake): ", Robot.inputs.isObjectPresentIntake());
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
