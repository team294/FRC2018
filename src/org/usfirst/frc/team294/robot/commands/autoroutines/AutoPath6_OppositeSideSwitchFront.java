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
		int angleMultiplier = 1;
		switch (startPosition) {
		case Left:
			angleMultiplier = 1;
			break;
			
		case Right:
			angleMultiplier = -1;
			break;
		}
		
		addSequential(new DriveStraightDistanceProfile(-53, 0, 100, 100));
		addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(-184, 90 * angleMultiplier, 150, 150));
		addSequential(new TurnGyro(180 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(47, 180 * angleMultiplier));
		
		
	}
}
