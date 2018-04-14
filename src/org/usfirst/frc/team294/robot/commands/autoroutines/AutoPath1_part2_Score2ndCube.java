package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPath1_part2_Score2ndCube extends CommandGroup {

	public AutoPath1_part2_Score2ndCube(StartingPosition startPosition) {
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

		// Assume we have the cube, so go back to scale to score
		addParallel(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, false));//true));
		addSequential(new DriveStraightDistanceProfile(-48, -15 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration));   // Was 5 degrees
		addSequential(new TurnGyro( 15.0 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, false));//true)); // enforce the arm being up before shooting
		addSequential(new CubeShootOut());
//		addSequential(new CubeLetGo());
		
		// Leave arm up, ready for teleop
//		addSequential(new ArmMoveWithPiston(RobotMap.armScaleLowPos, false));//true)); // enforce the arm being up before shooting
		addSequential(new ArmMoveWithPiston(90, RobotMap.PistonPositions.Null));//true)); // move arm to +90, since the arm will stay there when disabled

	}
}
