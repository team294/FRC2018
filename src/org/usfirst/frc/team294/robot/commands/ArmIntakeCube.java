package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmIntakeCube extends Command {

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
    	Robot.claw.clawCloseIfPhotoSwitch(); // Close the claw on the arm when the photoswitch is triggered
    	SmartDashboard.putBoolean("Arm Smart Close Return", Robot.claw.clawCloseIfPhotoSwitch());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !Robot.claw.getBumpSwitch(); //return once bump switch is pressed
    }

    // Called once after isFinished returns true
    protected void end() {
    	//stop claw motors
    	Robot.claw.setClawMotorPercent(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//stop claw motors
    	//Robot.claw.setClawMotorPercent(0.0);
    }
}
