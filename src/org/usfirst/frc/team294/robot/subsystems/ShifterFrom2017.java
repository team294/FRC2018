package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShifterFrom2017 extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

 
private final Solenoid shifter = new Solenoid(RobotMap.pnuematicShifter);


/**
 * Shift the gears up
 */
public void shiftUp() {
	shifter.set(true);
}

/**
 * * Shift the gears down
 * Set the gear piston to in
 */
public void shiftDown() {
	shifter.set(false);		
}

/**
 * Returns the state of the shifter
 * @return true for high gear, false for low
 */
public boolean isShifterInHighGear() {				// Is this needed or used anywhere???
	return shifter.get();
}

public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand(new MySpecialCommand());
}
}
