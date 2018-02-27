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
		int angleMultiplier = 1;
		switch (startPosition) {
		case Left:
			angleMultiplier = 1;
			break;
		case Right: 
			angleMultiplier = -1;
			break;			
		default:
			break;
		}
		
		addSequential(new DriveStraightDistanceProfile(-154, 0, 150, 150));
		addSequential(new TurnGyro(-90 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(26, -90 * angleMultiplier));
	}
}
