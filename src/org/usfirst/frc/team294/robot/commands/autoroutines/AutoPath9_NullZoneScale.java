package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.ArmMoveToDestAngle;
import org.usfirst.frc.team294.robot.commands.ArmMoveWithIntake;
import org.usfirst.frc.team294.robot.commands.ArmMoveWithPiston;
import org.usfirst.frc.team294.robot.commands.ArmPistonSmartExtendInDestZone;
import org.usfirst.frc.team294.robot.commands.ClawSetMotorSpeed;
import org.usfirst.frc.team294.robot.commands.CubeShootOut;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath9_NullZoneScale extends CommandGroup {

	public AutoPath9_NullZoneScale(StartingPosition startPosition) {
		int angleMultiplier = 1;

		switch (startPosition) {
		case Left:
			angleMultiplier = 1;
			break;
		case Right:
			angleMultiplier = -1;
			break;
		}
		addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentDefault));
		addSequential(new DriveStraightDistanceProfile(-162, 0, 100, 100));
		addParallel(new ArmPistonSmartExtendInDestZone(RobotMap.armScaleLowPos));
		addSequential(new DriveStraightDistanceProfile(-162, 0, 100, 100));
		addSequential(new TurnGyro(-90 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new CubeShootOut(RobotMap.clawPercentShootOut));
		addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos, RobotMap.PistonPositions.Null));
	}
}