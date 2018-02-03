package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToAndExtend extends Command {
	double destAngle;
	boolean finalPistonPosition;
    public ArmMoveToAndExtend(double destAngle, boolean finalPistonPosition) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.protoArmMotor);
    	requires(Robot.protoArmPiston);
    	this.destAngle = destAngle;
    	this.finalPistonPosition = finalPistonPosition;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	destAngle = destAngle%360;
    	if(destAngle > RobotMap.Ang4)destAngle = RobotMap.Ang4;
    	if(destAngle < RobotMap.Ang0)destAngle = RobotMap.Ang0;
    	if(!finalPistonPosition) {
    		Robot.protoArmPiston.retractMajorPiston();
    		Robot.protoArmPiston.retractMinorPiston();
    	}
    	
    	Robot.protoArmMotor.setArmAngle(destAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
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
