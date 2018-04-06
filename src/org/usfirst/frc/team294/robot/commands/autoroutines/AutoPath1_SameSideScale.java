package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
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
		addParallel(new ClawSetMotorSpeed(RobotMap.clawPercentDefault));
		addSequential(new WaitCommand(0.1));
		addParallel(new ArmMoveWithIntakeBack());

		// Drive to scale
		addSequential(new DriveStraightDistanceProfile(-246, 6 * angleMultiplier, 100, 80));  // was -244 in but that wasn't far enough  -244, 6, 100, 100
		addParallel(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, true));
		
		// Turn then shoot cube in scale
		addSequential(new TurnGyro(30 * angleMultiplier, TurnGyro.Units.Degrees));
		 addSequential(new WaitCommand(0.1));
		addSequential(new CubeShootOut(0.75));  // Was 0.66
		//addSequential(new WaitCommand(.25));
		
		// Load 2nd cube
//		addParallel(new LoadCubeSequence());
		addParallel(new LoadCubeSequenceWithIntakeOpenAuto());
		addSequential(new DriveStraightDistanceProfile(20, 0 * angleMultiplier, 100, 80));  
		addSequential(new TurnGyro(-75 * angleMultiplier, TurnGyro.Units.Degrees));  
		addSequential(new WaitCommand(.5));
		//addSequential(new TurnGyro());
		//addParallel(new IntakeCube());
		addSequential(new DriveStraightDistanceProfile(22, -75 * angleMultiplier, 100, 80));   // was 34, -75, 100, 80.  Shortened 12 in due to Match 35
		addSequential(new TurnGyro(0 * angleMultiplier, TurnGyro.Units.Degrees));
		
		// Turn towards closest cube using vision, if vision is working and we see a cube (otherwise do nothing and continue sequence)
		// Commented out, since RPi was removed from robot
		// IMPORTANT: If you use vision, then change the angle of the next DriveStraight = 9999 (use current heading)
//		addSequential(new TurnGyro());

		// Final movement foward to grab 2nd cube
		addSequential(new DriveStraightDistanceProfile(15, 0, 100, 100));   // Was 32 in
		addSequential(new WaitCommand(.75));

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

		// If we have the cube in the claw, then go back to scale and score
    	addSequential(new ConditionalCommand(new AutoPath1_part2_Score2ndCube(startPosition)) {
			protected boolean condition() {
				return (Robot.claw.getPhotoSwitch());
			}
		});
    		
/* Moved this to AutoPath1_part2_Score2ndCube()
		// Assume we have the cube, so go back to scale to score
		addParallel(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, false));//true));
		addSequential(new DriveStraightDistanceProfile(-48, 0 * angleMultiplier, 100, 100));
		addSequential(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, false));//true)); // enforce the arm being up before shooting
		addSequential(new CubeShootOut());
		
		// Leave arm up, ready for teleop
//		addSequential(new ArmMoveWithPiston(RobotMap.armScaleLowPos, false));//true)); // enforce the arm being up before shooting
		addSequential(new ArmMoveWithPiston(90, false));//true)); // move arm to +90, since the arm will stay there when disabled
*/
	}
}
