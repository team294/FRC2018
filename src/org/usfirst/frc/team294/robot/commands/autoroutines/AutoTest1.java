package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * 
 */
public class AutoTest1 extends CommandGroup {

    public AutoTest1() {

    	addSequential(new DriveStraightDistanceProfile(100,0));
    	addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
    	addSequential(new DriveStraightDistanceProfile(40,-90));
    	addSequential(new TurnGyro(-180, TurnGyro.Units.Degrees));
    	addSequential(new DriveStraightDistanceProfile(20,-180));
    }
}
