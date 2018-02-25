package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath6_OppositeSideSwitchFront extends CommandGroup {

	public AutoPath6_OppositeSideSwitchFront(StartingPosition startPosition) {
		int angleMultiplier = 0;
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
			angleMultiplier = 1;
			break;
			
		case Right:
			angleMultiplier = -1;
			break;
		}
		
		addSequential(new DriveStraightDistanceProfile(-53, 0, 100, 100));
		addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(-184, 90 * angleMultiplier, 150, 150));
		addSequential(new TurnGyro(180, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(47, 180 * angleMultiplier));
		
		
	}
}
