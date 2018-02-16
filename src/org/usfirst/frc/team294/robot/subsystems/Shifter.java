package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Gear shifter for the drive train
 */
public class Shifter extends Subsystem {

	private final DoubleSolenoid shifter = new DoubleSolenoid(RobotMap.pnuematicShifterLow,
			RobotMap.pnuematicShifterHigh);

	public Shifter() {
		super();
	}

	/**
	 * Shifts according to parameter
	 * 
	 * @param high
	 *            true for high gear, false for low gear
	 */
	public void setShift(boolean high) {
		shifter.set(high ? Value.kReverse : Value.kForward);
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
