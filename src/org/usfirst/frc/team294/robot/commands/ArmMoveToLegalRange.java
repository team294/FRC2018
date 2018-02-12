package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmMoveToLegalRange extends Command {

	private double currentAng;
	
	public ArmMoveToLegalRange() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);\
		requires(Robot.armMotor);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		currentAng = Robot.armMotor.getArmDegrees();
		currentAng = (currentAng < RobotMap.minAngle) ? RobotMap.minAngle + 5 : currentAng;// the fives are for buffer zone
		currentAng = (currentAng > RobotMap.maxAngle) ? RobotMap.maxAngle - 5: currentAng;
		SmartDashboard.putNumber("Desired Angle", currentAng);
		Robot.armMotor.setArmAngle(currentAng);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return (RobotMap.minAngle <= Robot.armMotor.getArmDegrees()) && (Robot.armMotor.getArmDegrees() <= RobotMap.maxAngle); 
		
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
