package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * intakes a cube and closes intake jaws when appropriate
 */
public class IntakeCube extends Command {

	boolean done = false;
	
    public IntakeCube() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setIntakeDeploy(true);
    	Robot.intake.setIntakeMotorPercent(RobotMap.intakePercentIn);
    	done = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	done = Robot.intake.smartCloseIntake();
    	SmartDashboard.putBoolean("Smart Close Intake Return", done);
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
    	Robot.intake.stop();
    }
}
