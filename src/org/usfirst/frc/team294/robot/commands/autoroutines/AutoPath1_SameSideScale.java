package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmPositions;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;
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
//		addSequential(new WaitCommand(0.1));
		addParallel(new ArmMoveWithPiston(RobotMap.armSwitchPosHigh, RobotMap.PistonPositions.Null)); //was armScaleLowPos

		// Drive to scale
		addSequential(new DriveStraightDistanceProfile(-246, 5 * angleMultiplier, RobotMap.maxSpeed, RobotMap.maxAcceleration));  // was 6 degrees
		addParallel(new ArmMoveWithPiston(RobotMap.armScaleBackwardsPos, false));
		
		// Turn then shoot cube in scale
		addSequential(new TurnGyro(25 * angleMultiplier, TurnGyro.Units.Degrees));   // Angle was 30deg, changing to 25 for 1st Qual match
		addSequential(new WaitCommand(0.5));
		addSequential(new CubeShootOut(1.0));  // Was 0.66
		
		// Load 2nd cube
		addParallel(new LoadCubeSequence(ArmPositions.Intake, PistonPositions.Extended, false));
		addSequential(new TurnGyro(-12 * angleMultiplier, TurnGyro.Units.Degrees));

		
		// Drive absolute angle (don't use vision)
//		addSequential(new DriveStraightDistanceProfile(54, -12 * angleMultiplier, 35, 70));
		
		// Drive using vision
		addSequential(new TurnGyro());		// turn using vision camera
		addSequential(new DriveStraightDistanceProfile(54, 9999, 35, 70));  // Drive straight with the direction from vision (angle  = 9999)

		// If we have the cube in the claw, then go back to scale and score
    	addSequential(new ConditionalCommand(new AutoPath1_part2_Score2ndCube(startPosition)) {
			protected boolean condition() {
				return (Robot.claw.getPhotoSwitch());
			}
		}); 
	} 
}
