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
		
		// Go to drive config
		addParallel(new ClawSetMotorSpeed(-1.0));
		addSequential(new WaitCommand(0.1)); 
		addParallel(new ClawSetMotorSpeed(-0.4));
		addParallel(new ArmMoveWithIntake());
		
		// Go forward to switch
		addSequential(new DriveStraightDistanceProfile(10, 0, 100, 100));
		if (goLeft) {
			addSequential(new DriveStraightDistanceProfile(105, 45 * angleMultiplier, 100, 100)); // acceleration factor was 70 
		} else {
			addSequential(new DriveStraightDistanceProfile(95, 40 * angleMultiplier, 100, 100));  // acceleration factor was 70 
		}
		addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(18, 0, 100, 100));
		
		// Shoot out cube
		addSequential(new AutoSwitchShoot());
		
		// Back up to get next cube
		addSequential(new DriveStraightDistanceProfile(-23, 0, 100, 100)); //should be -8 for middle cube, -23 for front cube

		// Lower arm and turn towards cube pile 
		addParallel(new LoadCubeSequence());  // was WithIntakeOpen
		//			addSequential(new WaitCommand(5.0));			
		addSequential(new TurnGyro(-90 * angleMultiplier, TurnGyro.Units.Degrees));
		//			addSequential(new WaitCommand(0.2));
		
		// Drive towards cubes and pick one up
/*		The following code drives forward to pick up the 2nd cube
		addSequential(new DriveStraightDistanceProfile(65, -90 * angleMultiplier, 65, 60));  // front cube:  65 in, 55 ips, 100 ips2
		addSequential(new WaitCommand(0.25)); // was 0.5	
*/	
	/*
		// the following code is used to score the second cube in the switch after intaking it
		// Raise arm and go back to our side of the switch
		addParallel(new ArmMoveWithIntake());
		addSequential(new DriveStraightDistanceProfile(-60, -90 * angleMultiplier, 100, 100));  // front cube:  -60 in, acceleration factor was 100
		
		// Turn to switch, go forward, and shoot
		addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(30, 0, 100, 100));  // front cube:  30 in, acceleration factor was 100
		addSequential(new AutoSwitchShoot());
	*/
	}
}
