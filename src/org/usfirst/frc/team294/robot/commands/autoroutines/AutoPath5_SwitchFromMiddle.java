package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
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

		
		// Go to drive config
		addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentInFully));
		addParallel(new IntakeSetDeploy(false));
		addSequential(new WaitCommand(0.1)); 
		addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentDefault));
		addParallel(new ArmMoveWithIntake());
//		
//		// Go forward to switch
//		addSequential(new DriveStraightDistanceProfile(10, 0, 100, 100));
		if (goLeft) {
			addSequential(new DriveStraightDistanceProfile(118, 26 * angleMultiplier, 100, 100)); // acceleration factor was 7, distance was 110
			
			
		} else { // going right
			addSequential(new DriveStraightDistanceProfile(108, 19 * angleMultiplier, 100, 100));  // acceleration factor was 70, distance was 100	
		}
		
			addSequential(new AutoSwitchShoot());
			addSequential(new DriveStraightDistanceProfile(-105, 23 * angleMultiplier, 100, 100));  // acceleration factor was 70 
			addParallel(new LoadCubeSequence());	
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			//addSequential(new TurnGyro()); 
//			addParallel(new LoadCubeSequenceWithIntakeOpenAuto());	// open claw to allow for more error on position
			addParallel(new LoadCubeSequence());
			//addParallel(new IntakeSetOpen(true));		// This was needed to force orpen intake,  it closed before moving forward
			addSequential(new DriveStraightDistanceProfile(60, 0, 100, 100));
			if (Robot.claw.getPhotoSwitch()) {
				addSequential(new ArmIntakeCube());	// The cube isn't in!  This won't terminate if there isn't a cube, so we will be in position to pick up a cube and not waste time
			}
			addParallel(new ArmMoveWithIntake());
			addSequential(new DriveStraightDistanceProfile(-60, 0, 100, 100));
			addSequential(new TurnGyro(22 * angleMultiplier, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(108, 25 * angleMultiplier, 100, 100));  // acceleration factor was 70 
			addSequential(new AutoSwitchShoot());
		
/*		addSequential(new DriveStraightDistanceProfile(-50, 20, 100, 100));
		addSequential(new TurnGyro(45, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(10, 20, 100, 100));		
		addSequential(new TurnGyro(-45, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(50, 20, 100, 100));
		addSequential(new AutoSwitchShoot()); */
		
//		addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));	
//		addSequential(new WaitCommand(0.2));
//		addSequential(new DriveStraightDistanceProfile(40, 0, 100, 100));
//		addSequential(new DriveStraightDistanceProfile(-10, 0, 100, 100));
//		addSequential(new TurnGyro(45, TurnGyro.Units.Degrees));
//		addSequential(new DriveStraightDistanceProfile(30, 45*angleMultiplier, 100, 100));
	}
}
