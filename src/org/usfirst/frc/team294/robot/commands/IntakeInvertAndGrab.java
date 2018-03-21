package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */

public class IntakeInvertAndGrab extends CommandGroup {

    public IntakeInvertAndGrab() {
        addSequential(new IntakeMotorsSetOpposite());
		addSequential(new WaitCommand(0.2));
		addParallel(new IntakeCube());
    	addSequential(new ArmIntakeCube());
    	addSequential(new WaitCommand(0.5));
    	addSequential(new IntakeSetSpeed(RobotMap.intakePercentOut));
    }
}
