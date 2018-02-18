package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath1_SameSideScale extends CommandGroup {

	public AutoPath1_SameSideScale(StartingPosition startPosition) {
		switch (startPosition) {
		case Left:
			addSequential(new DriveStraightDistanceProfile(252,  5)); //265
			addSequential(new TurnGyro(45 , TurnGyro.Units.Degrees));
			// put arm command here
			break;
		case Right:
////			addSequential(new TurnGyro(-15 , TurnGyro.Units.Degrees));
//			addSequential(new DriveStraightDistanceProfile(252,  -5)); //265
//			addSequential(new TurnGyro(-45 , TurnGyro.Units.Degrees));
//			// Score Cube
//			addSequential(new TurnGyro(-120, TurnGyro.Units.Degrees));
//			addSequential(new DriveStraightDistanceProfile(50, -120));
//			addSequential(new TurnGyro(-180, TurnGyro.Units.Degrees));
//			addSequential(new DriveStraightDistanceProfile(20, -180));
//			// Grab Cube
//			addSequential(new DriveStraightDistanceProfile(-20, -115));
//			addSequential(new DriveStraightDistanceProfile(-20, -225));
//			// Score Cube 2 Backwards
			
			addSequential(new DriveStraightDistanceProfile(-252, -5));
			addSequential(new TurnGyro(-45, TurnGyro.Units.Degrees));
			// Score Cube
			addSequential(new TurnGyro(45, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(30, 45));
			addSequential(new DriveStraightDistanceProfile(15, 0));
			// Grab cube
			addSequential(new DriveStraightDistanceProfile(-20, 45));
//			addSequential(new TurnGyro(-45, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-60, -45));
			// Dunk cube

			break;
		}
	}
}
