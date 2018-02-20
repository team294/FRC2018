package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.ArmMoveToDestAngle;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.utilities.AutoSelection.StartingPosition;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units.*;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath2_OppositeSideScale extends CommandGroup {

    public AutoPath2_OppositeSideScale(StartingPosition startPosition) {
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
		addParallel(new ArmMoveToDestAngle(90)); // TODO Change Angle
    	switch (startPosition) {
		case Left:
			addSequential(new DriveStraightDistanceProfile(-200, 0, 150, 150));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-180, 90, 150, 150));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-50, -45)); // Drive and turn to -45 in one swoop
			// Backwards dunk cube
			addSequential(new DriveStraightDistanceProfile(50, 0)); // Drive back to "normal" angle
			// Do second cube?
			break;
		case Right:
			addSequential(new DriveStraightDistanceProfile(-200, 0, 150, 150));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-180, -90, 150, 150));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-50, 45)); //drive and turn to 45 degrees in one swoop
			//backwards dunk cube
			addSequential(new DriveStraightDistanceProfile(50, 0)); //drive back to normal angle
			// do second cube?
			break;

		}

	}
}
