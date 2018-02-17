package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.ArmMoveToDestAngle;
import org.usfirst.frc.team294.robot.commands.ClawOpen;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath2_OppositeSideScale extends CommandGroup {

	public AutoPath2_OppositeSideScale(int startPosition) {
		addParallel(new ArmMoveToDestAngle(90)); // TODO Change Angle
		switch (startPosition) {
		case 1:
			addSequential(new DriveStraightDistanceProfile(-200, 0));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-180, 90));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-50, -45)); // Drive and turn to -45 in one swoop
			// Backwards dunk cube
			addSequential(new DriveStraightDistanceProfile(50, 0)); // Drive back to "normal" angle
			// Do second cube?
			break;
		case 3:
			addSequential(new DriveStraightDistanceProfile(-200, 0));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-180, -90));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-50, 45)); //drive and turn to 45 degrees in one swoop
			//backwards dunk cube
			addSequential(new DriveStraightDistanceProfile(50, 0)); //drive back to normal angle
			// do second cube?
			break;

		}

	}
}
