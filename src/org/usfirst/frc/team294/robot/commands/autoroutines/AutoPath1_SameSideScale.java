package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath1_SameSideScale extends CommandGroup {

	public AutoPath1_SameSideScale(int startPosition) {

		// Add Commands here:
		// e.g. addSequential(new Command1());
		// addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		// addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
		switch (startPosition) {
		case 1:
			addSequential(new DriveStraightDistanceProfile(300, 0));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(25, 90));
			// put arm command here
			break;
		case 3:
			addSequential( new DriveStraightDistanceProfile(300,  0));
			addSequential(new TurnGyro(-90 , TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(25, -90));
			// put arm command here 
			break;

		}
	}
}
