package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class IntakeInvertAndGrab extends CommandGroup {

    public IntakeInvertAndGrab() {
        addSequential(new IntakeMotorsSetOpposite());
		addSequential(new WaitCommand(0.35));
		addSequential(new IntakeCube());
    }
}
