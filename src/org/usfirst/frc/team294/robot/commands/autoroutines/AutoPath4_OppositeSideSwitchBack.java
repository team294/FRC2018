package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;

import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath4_OppositeSideSwitchBack extends CommandGroup {

	public AutoPath4_OppositeSideSwitchBack(StartingPosition startPosition) {

		int angleMultiplier = 1;
		switch (startPosition) {
		case Left:
			angleMultiplier = 1;
			break;
		case Right:
			angleMultiplier = -1;
			break;
		}
		
		addSequential(new DriveStraightDistanceProfile(-230, 0, 150, 150));
		addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));
		
		if (Robot.robotPrefs.inBCRLab) {
			// If on left, then just stop since we are at the beam
			if (startPosition == StartingPosition.Left) return;

			// If on right, then drive short because of beam then stop
			addSequential(new DriveStraightDistanceProfile(-152, 90 * angleMultiplier, 150, 150)); // Actually go 165" but also
			return;
		}
		
		addSequential(new DriveStraightDistanceProfile(-200, 90 * angleMultiplier, 150, 150));
		addSequential(new TurnGyro(0 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(50, 0));
		addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(20, 90 * angleMultiplier)); // Touch the Switch wall
		// Score cube

	}
}
