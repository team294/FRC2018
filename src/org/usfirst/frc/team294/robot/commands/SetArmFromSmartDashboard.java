package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.utilities.*;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetArmFromSmartDashboard extends Command  {
	private double currAngle;
	public ToleranceChecker tolcheck;
	private double targetAngle;
	
    public SetArmFromSmartDashboard() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.armMotor);
    	tolcheck = new ToleranceChecker(3, 10);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
		double targetAngle = SmartDashboard.getNumber("set Arm Angle", 0);
		this.targetAngle = targetAngle;
    	Robot.armMotor.startPID(targetAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		Robot.armMotor.armUpdatePID();
		currAngle = Robot.armMotor.getArmDegrees();
		
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
//    	return tolcheck.success(Math.abs(targetAngle - currAngle));
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
