package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonSetBothStates extends Command {

	PistonPositions state;
	boolean moving = false;
	
	/**
	 * Sets the state of both pistons
	 * @param state RobotMap.PistonPositions. Values other then Extended (including Null and Moving) will retract pistons
	 */
    public ArmPistonSetBothStates(PistonPositions state) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.armPiston);
    	this.state = state;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if (state == PistonPositions.Extended) moving = Robot.armPiston.smartExtend();
    	else Robot.armPiston.smartRetract();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (moving) return Robot.armPiston.getMajor() != PistonPositions.Moving; /* && Robot.armPiston.getMinor() != PistonPositions.Moving; */
    	return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}