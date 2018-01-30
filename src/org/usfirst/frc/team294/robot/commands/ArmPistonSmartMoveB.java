package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmPistonSmartMoveB extends CommandGroup {

	/**
	 * Moves the arm and extends/retracts the pistons as needed.
	 * This sub-sequence assumes that we are not moving the arm out of
	 * the HigH or Low regions.
	 * @param destAngle Desired arm angle to go to (0 = horizontal, 90 = vertical up)
	 * @param extendPistonAtEnd true = extend piston (when legal), false = retract piston
	 */
    public ArmPistonSmartMoveB(double destAngle, boolean extendPistonAtEnd) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
