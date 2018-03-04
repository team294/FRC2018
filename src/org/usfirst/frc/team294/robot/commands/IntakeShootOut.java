package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeShootOut extends Command {

	private double leftPercent = 100; // may want to have different speeds
	private double rightPercent = 100;
	private double timeShot = 1000;
	
    public IntakeShootOut() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setIntakeMotorPercent(leftPercent);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.setIntakeMotorPercent(leftPercent);
		if (timeShot == 1000) {
			timeShot = Timer.getFPGATimestamp();
		} else {
		}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if ((!Robot.claw.getPhotoSwitch() && Timer.getFPGATimestamp() >= timeShot + 2) || Timer.getFPGATimestamp() >= timeShot + 3) {
			end();
			timeShot = 1000;
			Robot.intake.updateCubeStatus();
			return true;
		} else {
			return false;
		}
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.intake.setIntakeMotorPercent(RobotMap.intakePercentOut);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
