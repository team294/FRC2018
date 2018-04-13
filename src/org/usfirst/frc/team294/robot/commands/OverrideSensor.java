package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import java.lang.ref.Reference;
import java.lang.ref.SoftReference;

import org.usfirst.frc.team294.robot.Robot;
/**
 *
 */
public class OverrideSensor extends Command {
	public static enum Sensors{
		Bump,
		ArmPiston,	
	}

	private Sensors sensor;
	
    public OverrideSensor(Sensors sensor) {
    	this.sensor = sensor;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	switch(sensor) {
    	case Bump:
    		Robot.claw.overrideBumpSwitch();
    		break;
    	case ArmPiston:
    		Robot.armPiston.overrideArmSensor();
    		break;
    	}
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
