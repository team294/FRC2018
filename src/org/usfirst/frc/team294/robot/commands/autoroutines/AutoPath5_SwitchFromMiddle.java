package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
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
		
		// Go forward to switch
//		addSequential(new DriveStraightDistanceProfile(10, 0, 100, 100));
		if (goLeft) {
			addSequential(new DriveStraightDistanceProfile(118, 26 * angleMultiplier, 100, 100)); // acceleration factor was 7, distance was 110
			
			
		} else { // going right
			addSequential(new DriveStraightDistanceProfile(108, 19 * angleMultiplier, 100, 100));  // acceleration factor was 70, distance was 100	
		}
		
		// Shoot out first cube
		addSequential(new AutoSwitchShoot());
		
		// Back away from switch to front of cube pile
		addSequential(new DriveStraightDistanceProfile(-105, 23 * angleMultiplier, 100, 100));  // acceleration factor was 70 
		
		// Start intake and drive forward to grab 2nd cube
		addParallel(new LoadCubeSequence());
//		addParallel(new LoadCubeSequenceWithIntakeOpenAuto());
		
		addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
		//addSequential(new TurnGyro()); 
		//			addParallel(new LoadCubeSequenceWithIntakeOpenAuto());	// open claw to allow for more error on position
		//addParallel(new LoadCubeSequence());		// TODO: This was redundant, but need to check!!!!
		//addParallel(new IntakeSetOpen(true));		// This was needed to force orpen intake,  it closed before moving forward
		addSequential(new DriveStraightDistanceProfile(60, 0, 100, 100));
		
		// Back up so we can rotate cube if necessary
		addSequential(new DriveStraightDistanceProfile(-10, 0, 100, 100));
		
		// If we have the cube in the intake in diamond shape (not in claw), then try rotating the cube
    	addSequential(new ConditionalCommand(new IntakeInvertAndGrab()) {
			protected boolean condition() {
				return (Robot.intake.getPhotoSwitch() && !Robot.claw.getPhotoSwitch());
			}
		});
		// If we have the cube in the intake in diamond shape (not in claw), then try rotating the cube
    	addSequential(new ConditionalCommand(new IntakeInvertAndGrab()) {
			protected boolean condition() {
				return (Robot.intake.getPhotoSwitch() && !Robot.claw.getPhotoSwitch());
			}
		});

		// If we don't have the cube in the claw, then stop (well, wait 15 sec to end of auto mode)
    	addSequential(new ConditionalCommand(new WaitCommand(15)) {
			protected boolean condition() {
				return (!Robot.claw.getPhotoSwitch());
			}
		});

		// Move arm up to switch position and back away from pile
		addParallel(new ArmMoveWithIntake());
		addSequential(new DriveStraightDistanceProfile(-50, 0, 100, 100)); // distance was -60, but changed because we back up 10 inches earlier
		
		// Turn towards switch and drive to switch
		addSequential(new TurnGyro(22 * angleMultiplier, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(108, 25 * angleMultiplier, 100, 100));  // acceleration factor was 70 
		
		// Score 2nd cube at switch
		addSequential(new AutoSwitchShoot());

	}
}
