package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath6_OppositeSideSwitchFront extends CommandGroup {

	public AutoPath6_OppositeSideSwitchFront(int startPosition) {
		switch (startPosition) {
		case 1:
			addSequential(new DriveStraightDistanceProfile(-53, 0, 100, 100));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-184, 90, 150, 150));
			addSequential(new TurnGyro(180, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(47, 180, 100, 100));
			
// Old reference code
//			addSequential(new DriveStraightDistanceProfile(53, 0, 100, 100));
//			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
//			addSequential(new DriveStraightDistanceProfile(184, 90, 150, 150));
//			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
//			addSequential(new DriveStraightDistanceProfile(47, 0, 100, 100));
			// arm code 
			break;
		case 3:
			addSequential(new DriveStraightDistanceProfile(-53, 0, 100, 100));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-184, -90, 150, 150));
			addSequential(new TurnGyro(-180, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(47, -180, 100, 100));
			
// Old reference code
//			addSequential(new DriveStraightDistanceProfile(53, 0, 100, 100));
//			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
//			addSequential(new DriveStraightDistanceProfile(184, -90, 150, 150));
//			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
//			addSequential(new DriveStraightDistanceProfile(47, 0, 100, 100));
			// arm code 
			break;

		}
	}
}
