package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonSmartRetract extends Command {

    public ArmPistonSmartRetract(double destAngle, boolean pistonFinal) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.protoArmPiston);
    	requires(Robot.protoArmMotor);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	double currentAngle = Robot.protoArmMotor.getArmDegrees();
    	if(RobotMap.getArmZone(currentAngle) == ArmZones.Middle || RobotMap.getArmZone(currentAngle) == ArmZones.Backwards) {
    		// If arm is in the middle or backwards zone set pistons to retracted
    		Robot.protoArmPiston.setMinor(RobotMap.PistonPositions.Retracted);
    		Robot.protoArmPiston.setMajor(RobotMap.PistonPositions.Retracted);
    	}
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
