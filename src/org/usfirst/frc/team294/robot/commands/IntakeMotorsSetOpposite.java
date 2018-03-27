package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.WaitCommand;


/**
 *
 */
public class IntakeMotorsSetOpposite extends Command {

    public IntakeMotorsSetOpposite() {
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.stop();  // Set to 0 volts to start current test at 0
//    	new WaitCommand(0.25);			// insure that a new current is read  // This doesn't do anything.  Creating a command in a command won't execute the command
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (timeSinceInitialized()>0.15) {
    		Robot.intake.setIntakeMotorPercentOpposite();
    		Robot.intake.logMotorCurrents();
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Robot.intake.isCurrentDecreasing()) {
    		Robot.intake.stop();
    		return true;
    	} else {
    		return false;
    	}
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
