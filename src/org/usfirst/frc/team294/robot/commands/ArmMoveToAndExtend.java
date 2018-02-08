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
	boolean pistonDone;
	boolean armDone;
	double currentAngle = Robot.protoArmMotor.getArmDegrees();
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
    	if(destAngle > RobotMap.maxAngle)destAngle = RobotMap.maxAngle;
    	if(destAngle < RobotMap.minAngle)destAngle = RobotMap.minAngle;
    	if(!finalPistonPosition) {
    		Robot.protoArmPiston.retractMajorPiston();
    		Robot.protoArmPiston.retractMinorPiston();
    		pistonDone=true;
    	}
    	if (!((destAngle < RobotMap.upperBound && destAngle > RobotMap.middleBound)
				|| (destAngle < RobotMap.lowerBound && destAngle > RobotMap.minAngle))) {
    		Robot.protoArmPiston.retractMajorPiston();
    		Robot.protoArmPiston.retractMinorPiston();
    		pistonDone = true;
		}
    	Robot.protoArmMotor.setArmAngle(destAngle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	currentAngle = Robot.protoArmMotor.getArmDegrees();
    	if (destAngle < RobotMap.upperBound && destAngle > RobotMap.middleBound && !pistonDone) {
			if (currentAngle < RobotMap.upperBound && currentAngle > RobotMap.middleBound) {
				Robot.protoArmPiston.extendMinorPiston();
				Robot.protoArmPiston.extendMajorPiston();
				pistonDone = true;
			}
		} else if (destAngle < RobotMap.lowerBound && destAngle > RobotMap.minAngle && !pistonDone) {
			if (currentAngle < RobotMap.lowerBound && currentAngle > RobotMap.minAngle) {
				Robot.protoArmPiston.extendMinorPiston();
				pistonDone = true;
			}
		}
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
