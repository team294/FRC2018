package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath2_OppositeSideScale extends CommandGroup {

    public AutoPath2_OppositeSideScale(int startPosition) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	switch (startPosition) {
		case 1:
			addSequential(new DriveStraightDistanceProfile(200, 0));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(180, 90));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(100, 0));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));

			
			// put arm command here
			break;
		case 3:
			addSequential(new DriveStraightDistanceProfile(200, 0));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(180, -90));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential( new DriveStraightDistanceProfile(100, 0));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			
			// put arm command here 
			break;

		}

    }
}