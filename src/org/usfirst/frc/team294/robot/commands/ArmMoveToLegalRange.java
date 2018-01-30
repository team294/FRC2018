package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToLegalRange extends Command {

	private double Ang0; // arm cannot extend downward past this angle
	private double Ang1; // piston1 can be extended between Ang0 and Ang1, cube picked up below Ang1
	private double Ang2; // arm cannot extend between Ang1 and Ang2
	private double Ang3; // both pistons can be extended between Ang2 and Ang 3
	private double Ang4; // arm cannot extend upward past this angle
	private double currentAng = Robot.protoArmMotor.getArmDegrees();
	private double destAng;
	private boolean currentPiston; // true = extended, false = retracted
	private boolean pistonFinal;
	private boolean isLegal = false;

	public ArmMoveToLegalRange() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);\
		requires(Robot.protoArmMotor);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		currentAng = (currentAng < Ang0) ? Ang0 : currentAng;
		currentAng = (currentAng > Ang4) ? Ang4 : currentAng;
		Robot.protoArmMotor.setArmAngle(currentAng);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (Ang0 < currentAng) || (currentAng < Ang4);
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
