package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 *
 */
public class Inputs extends Subsystem {
	
	private final DigitalInput photoSwitchClaw = new DigitalInput(RobotMap.photoSwitchClaw);
	private final DigitalInput bumpSwitch = new DigitalInput(RobotMap.bumpSwitch);
	private final DigitalInput photoSwitchIntake = new DigitalInput(RobotMap.photoSwitchIntake);

	public Inputs() {
		super();
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
	
	public boolean isObjectPresentIntake() {
		return photoSwitchIntake.get();
	}
	
	public boolean isObjectPresentClaw() {
		return photoSwitchClaw.get();
	}
	
	public boolean isCubeFullyIn() {
		return bumpSwitch.get();
	}
}
