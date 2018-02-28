package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPath1_SameSideScale extends CommandGroup {

	public AutoPath1_SameSideScale(StartingPosition startPosition) {
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
		addParallel(new IntakeSetDeploy(true));
		addSequential(new DriveStraightDistanceProfile(-246, 7*angleMultiplier, 95, 150));
		addParallel(new ArmMoveWithPiston(110, true));
		addSequential(new TurnGyro(45 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new WaitCommand(2));
		addSequential(new CubeShootOut());
		
		// Score cube 1
		
//		addSequential(new TurnGyro(-45 * angleMultiplier, TurnGyro.Units.Degrees));
//		addSequential(new DriveStraightDistanceProfile(30, -45 * angleMultiplier));
//		// TODO We should TurnGyro before going from 45 degrees to 0 degrees
//		addSequential(new DriveStraightDistanceProfile(15, 0));  
//		// TODO We should TurnGyro before going from 0 degrees to 45 degrees
//		
//		// Grab second cube
//		
//		addSequential(new DriveStraightDistanceProfile(-20, -45 * angleMultiplier));
//		addSequential(new DriveStraightDistanceProfile(-60, 45 * angleMultiplier ));
	}
}
