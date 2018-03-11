package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmIntakeCube extends Command {
	
	private double timeClawClosed = 1000.0;

    public ArmIntakeCube() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.claw);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//set the claw to intake
    	Robot.claw.openClaw();
    	Robot.claw.setClawMotorPercent(RobotMap.clawPercentIn);
    	//actuate claw jaws
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putBoolean("Arm Smart Close Return", Robot.claw.clawCloseIfPhotoSwitch());
    	
    	// Close the claw on the arm when the photoswitch is triggered
    	if (Robot.claw.clawCloseIfPhotoSwitch()) {
    		if(timeClawClosed == 1000) {
    			timeClawClosed = Timer.getFPGATimestamp();
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	// if the claw has been closed 
    	if (Robot.claw.getBumpSwitch() || Timer.getFPGATimestamp() >= (timeClawClosed + 1)) {
    		end();
    		timeClawClosed = 1000;
    		return true;
    	} else {
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    	//stop claw motors
    	Robot.claw.setClawMotorPercent(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    	//stop claw motors
    	//Robot.claw.setClawMotorPercent(0.0);
    }
}
