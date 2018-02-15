package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Gear shifter for the drive train
 */
public class Shifter extends Subsystem {

	
	private DoubleSolenoid shifter; 

	public Shifter() {
		super();
		if (Robot.prototypeRobot) shifter = new DoubleSolenoid(RobotMap.pnuematicShifterLow, RobotMap.pnuematicShifterHigh);
	}
	
	/**
	 * Shifts according to parameter
	 * @param high true for high gear, false for low gear
	 */
	public void setShift(boolean high) {
		shifter.set(high ? Value.kForward : Value.kReverse);
	}
	
	/**
	 * Shift the gears up
	 */
	public void shiftUp() {
		shifter.set(Value.kReverse); // kReverse is high gear
	}

	/**
	 * Set the gear piston to in
	 */
	public void shiftDown() {
		if (Robot.prototypeRobot) shifter.set(Value.kForward); // KForward is low gear
	}

	/**
	 * Returns the state of the shifter
	 * 
	 * @return true for high gear, false for low
	 */
	public boolean isShifterInHighGear() {
		return shifter.get().equals(Value.kForward);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
