package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath3_SameSideSwitch extends CommandGroup {

	public AutoPath3_SameSideSwitch(int startPosition) {
		switch (startPosition) {
		case 1:
			addSequential(new DriveStraightDistanceProfile(-154, 0));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(26, -90));
			// arm command
			break;
		case 3 : 
			addSequential(new DriveStraightDistanceProfile(-154, 0)); // it should go 157", but due to inconsistency we made it go 154"
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(26, 90));
			// arm command
			break;

		}
	}
}
