package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath1_SameSideScale extends CommandGroup {

	public AutoPath1_SameSideScale(int startPosition) {
		switch (startPosition) {
		case 1:
			addSequential(new DriveStraightDistanceProfile(-252, -5, 150, 150));
			addSequential(new TurnGyro(45, TurnGyro.Units.Degrees));
			// Score cube 1
			addSequential(new TurnGyro(-45, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(30, -45));
			addSequential(new DriveStraightDistanceProfile(15, 0));
			// Grab second cube
			addSequential(new DriveStraightDistanceProfile(-20, -45));
			addSequential(new DriveStraightDistanceProfile(-60, 45));
			// Dunk second cube
			break;
		case 3:
			addSequential(new DriveStraightDistanceProfile(-252, -5, 150, 150));
			addSequential(new TurnGyro(-45, TurnGyro.Units.Degrees));
			// Score Cube 1
			addSequential(new TurnGyro(45, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(30, 45));
			addSequential(new DriveStraightDistanceProfile(15, 0));
			// Grab second cube
			addSequential(new DriveStraightDistanceProfile(-20, 45));
			addSequential(new DriveStraightDistanceProfile(-60, -45));
			// Dunk second cube

			
			
			
				
	// Old reference code
	////		addSequential(new TurnGyro(-15 , TurnGyro.Units.Degrees));
	//		addSequential(new DriveStraightDistanceProfile(252,  -5)); //265
	//		addSequential(new TurnGyro(-45 , TurnGyro.Units.Degrees));
	//		// Score Cube
	//		addSequential(new TurnGyro(-120, TurnGyro.Units.Degrees));
	//		addSequential(new DriveStraightDistanceProfile(50, -120));
	//		addSequential(new TurnGyro(-180, TurnGyro.Units.Degrees));
	//		addSequential(new DriveStraightDistanceProfile(20, -180));
	//		// Grab Cube
	//		addSequential(new DriveStraightDistanceProfile(-20, -115));
	//		addSequential(new DriveStraightDistanceProfile(-20, -225));
	//		// Score Cube 2 Backwards
			break;
		}
	}
}
