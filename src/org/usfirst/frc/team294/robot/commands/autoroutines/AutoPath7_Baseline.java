package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * plz dont pass anything other than {1,2,3}
 */
public class AutoPath7_Baseline extends CommandGroup {

	public AutoPath7_Baseline(int startPosition) {
		switch (startPosition) {
		case 2:
			addSequential(new DriveStraightDistanceProfile(55, 0));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(60, -90));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(50, 0));
			break;
		default:
			addSequential(new DriveStraightDistanceProfile(120, 0));
		}
	}
}
