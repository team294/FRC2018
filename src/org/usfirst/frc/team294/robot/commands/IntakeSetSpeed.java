package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *opens the intake jaws and sets the motors to outtake
 */
public class IntakeSetSpeed extends Command {

	double percentPower;
	
    public IntakeSetSpeed(double percentPower) {
        this.percentPower = percentPower;
//    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
 //   	Robot.intake.setIntakeMotorPercent(percentPower); // Sets the intake motors to specified percent
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
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
