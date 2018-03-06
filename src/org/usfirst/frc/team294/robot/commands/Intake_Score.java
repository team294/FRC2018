package org.usfirst.frc.team294.robot.commands;


import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;


/**
 *
 */
public class Intake_Score extends CommandGroup {

    public Intake_Score() {
    	//addSequential(new IntakeSetDeploy(true)); // may want to add this if intake is up when running shoot command
        addParallel(new ClawSetMotorSpeed(0));
        addSequential(new IntakeCubeGrab());
//        addSequential(new IntakeSetDeploy(false));
        addSequential(new IntakeShootOut());
        

    }
}
