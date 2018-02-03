package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 *
 */
public class Inputs extends Subsystem {
	private final DigitalInput photoSwitch = new DigitalInput(RobotMap.photoSwitch);

	public Inputs() {
		super();
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public boolean isObjectPresent() {
		return photoSwitch.get();
	}
}
