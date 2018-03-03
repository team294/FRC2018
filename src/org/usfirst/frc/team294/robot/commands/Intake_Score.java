package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class Intake_Score extends CommandGroup {

    public Intake_Score() {
        addParallel(new ClawSetMotorSpeed(0));
        addSequential(new IntakeCubeGrab());
        addSequential(new IntakeSetDeploy(false));
        addSequential(new IntakeShootOut());
        
    }
}
