package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPath5_SwitchFromMiddle extends CommandGroup {
	int angleMultiplier;

	/**
	 * Robot starts in the middle and puts cube on switch.
	 * 
	 * @param goLeft
	 *            true = put cube on left plate, false = put cube on right plate
	 */
	public AutoPath5_SwitchFromMiddle(boolean goLeft) {
		if (goLeft) {
			angleMultiplier = -1;
		} else {
			angleMultiplier = 1;
		}
		addParallel(new ClawSetMotorSpeed(-1.0));
		addSequential(new WaitCommand(0.1)); 
		addParallel(new ClawSetMotorSpeed(-0.4));
		addParallel(new ArmMoveWithIntake());
		addSequential(new DriveStraightDistanceProfile(10, 0, 100, 100));
		if (goLeft) {
			addSequential(new DriveStraightDistanceProfile(95, angleMultiplier * 50, 100, 100));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(18, 0, 50, 50));
		} else {
			addSequential(new DriveStraightDistanceProfile(95, angleMultiplier * 40, 100, 70));  // acceleration factor was 100 
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(18, 0, 50, 50));
		}
		addSequential(new AutoSwitchShoot());
		addSequential(new DriveStraightDistanceProfile(-8, 0, 100, 100)); //should be -8
        if (goLeft) {
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(45, 45, 100, 100));
			addSequential(new ArmMoveWithPiston(-13.0, false));
			addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentIn));
			addParallel(new ArmIntakeCube());
			addSequential(new DriveStraightDistanceProfile(-10, -45, 100, 100));
			addSequential(new TurnGyro(-45, TurnGyro.Units.Degrees));
        } else {
			addParallel(new LoadCubeSequenceWithIntakeOpen());
//			addSequential(new WaitCommand(5.0));			
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(60, -90, 75, 100));  // speed was 100 
			addSequential(new WaitCommand(5.0));
//			addSequential(new ArmMoveWithPiston(-13.0, false));
//			addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentIn));
//			addParallel(new ArmIntakeCube());
			addParallel(new ArmMoveWithIntake());
			addSequential(new DriveStraightDistanceProfile(-50, -80, 100, 100));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
        }
		addSequential(new DriveStraightDistanceProfile(25, 0, 100, 100));
		addSequential(new AutoSwitchShoot());
	}
}
