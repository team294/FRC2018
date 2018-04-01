package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ClimbMotorTurn extends Command {
	private boolean done = false;

    public ClimbMotorTurn() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.climb);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
  
    }
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(DriverStation.getInstance().getMatchTime() <= 30 && Robot.armMotor.getArmDegrees() >= 75.0){
    		Robot.climb.setClimbMotors(0.1);
    		done = false;
    	}
    	else {
    		Robot.climb.setClimbMotors(0.0);
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
