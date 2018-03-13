package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.ArmMoveToDestAngle;
import org.usfirst.frc.team294.robot.commands.ClawSetMotorSpeed;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath2_OppositeSideScale extends CommandGroup {

    public AutoPath2_OppositeSideScale(StartingPosition startPosition) {
    	int angleMultiplier = 1; 

		addParallel(new ArmMoveToDestAngle(90)); // TODO Change Angle

		switch (startPosition) {
		case Left:
			angleMultiplier = 1; 
			break;
		case Right:
			angleMultiplier = -1;
			break;
		}
		addParallel(new ClawSetMotorSpeed(-0.40));
		addSequential(new DriveStraightDistanceProfile(-200, 0, 120, 150));
		addSequential(new TurnGyro(90 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(-180, 90 * angleMultiplier, 120, 150));
		addSequential(new TurnGyro(-45 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(-50, -45 * angleMultiplier)); 
		// Backwards dunk cube
		addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(50, 0)); // Drive back to "normal" angle
		// Do second cube?

	}
}
