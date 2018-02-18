package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath3_SameSideSwitch extends CommandGroup {

	public AutoPath3_SameSideSwitch(StartingPosition startPosition) {
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
		case Left:
			addSequential(new DriveStraightDistanceProfile(154, 0));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(26, 90));
			// arm command 
			break;
		case Right: 
			addSequential(new DriveStraightDistanceProfile(154, 0)); // it should go 157", but due to inconsistency we made it go 154"
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(26,  -90));
			// arm command 
			break;

		}
	}
}
