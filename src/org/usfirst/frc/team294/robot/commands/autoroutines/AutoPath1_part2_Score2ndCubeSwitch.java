package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.ArmMoveWithPiston;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.IntakeSetDeploy;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoPath1_part2_Score2ndCubeSwitch extends CommandGroup {

    public AutoPath1_part2_Score2ndCubeSwitch(StartingPosition startPosition) {
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
    		addParallel(new ArmMoveWithPiston(0, false));
    		addSequential(new DriveStraightDistanceProfile(-12, 0, 100, 100));
    		addSequential(new WaitCommand(0.1));
    		addParallel(new IntakeSetDeploy(false));
    		addSequential(new DriveStraightDistanceProfile(12, 0, 100, 100));
    		addSequential(new TurnGyro(-10 * angleMultiplier, TurnGyro.Units.Degrees));
    		addSequential(new AutoSwitchShoot());
    	}
    }
