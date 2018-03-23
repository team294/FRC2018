package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
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
		addParallel(new ClawSetMotorSpeed(-0.40));
		addSequential(new WaitCommand(0.1));
		addParallel(new ArmMoveWithIntakeBack());
		addSequential(new DriveStraightDistanceProfile(-245, 6 * angleMultiplier, 100, 100));
		addParallel(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, true));
		addSequential(new TurnGyro(30 * angleMultiplier, TurnGyro.Units.Degrees));
		 addSequential(new WaitCommand(0.1));
		addSequential(new CubeShootOut());
		//addSequential(new WaitCommand(.25));
		
		// Load 2nd cube
//		addParallel(new LoadCubeSequence());
		addParallel(new LoadCubeSequenceWithIntakeOpen());
		addSequential(new TurnGyro(-35 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new WaitCommand(.5));
		//addSequential(new TurnGyro());
		//addParallel(new IntakeCube());
		addSequential(new DriveStraightDistanceProfile(25, -35 * angleMultiplier, 100, 100));
		addSequential(new TurnGyro(0 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(33, 0, 100, 100));
		addSequential(new WaitCommand(.75));
		addParallel(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, false));//true));
		addSequential(new DriveStraightDistanceProfile(-48, 0 * angleMultiplier, 100, 100));
		addSequential(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, false));//true)); // enforce the arm being up before shooting
		addSequential(new CubeShootOut());
//		addSequential(new ArmMoveWithPiston(RobotMap.armScaleLowPos, false));//true)); // enforce the arm being up before shooting
		addSequential(new ArmMoveWithPiston(90, false));//true)); // move arm to +90, since the arm will stay there when disabled
		

		// Score cube 1

		// addSequential(new TurnGyro(-45 * angleMultiplier, TurnGyro.Units.Degrees));
		// addSequential(new DriveStraightDistanceProfile(30, -45 * angleMultiplier));
		// // TODO We should TurnGyro before going from 45 degrees to 0 degrees
		// addSequential(new DriveStraightDistanceProfile(15, 0));
		// // TODO We should TurnGyro before going from 0 degrees to 45 degrees
		//
		// // Grab second cube
		//
		// addSequential(new DriveStraightDistanceProfile(-20, -45 * angleMultiplier));
		// addSequential(new DriveStraightDistanceProfile(-60, 45 * angleMultiplier ));
	}
}
