package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */

public class IntakeInvertAndGrab extends CommandGroup {

    public IntakeInvertAndGrab() {
    	addSequential(new LogMessage("IntakeInvertAndGrab,0 start", true));
 //       addSequential(new IntakeMotorsSetOpposite());
    	addSequential(new LogMessage("IntakeInvertAndGrab,1 set opposite done", true));
		addSequential(new WaitCommand(0.2));
    	addSequential(new LogMessage("IntakeInvertAndGrab,2 0.2 sec done", true));
	//	addParallel(new IntakeCube());
    //	addSequential(new ArmIntakeCube());
    	addSequential(new LogMessage("IntakeInvertAndGrab,3 arm intake cube done", true));
    	addSequential(new WaitCommand(0.5));
    	addSequential(new LogMessage("IntakeInvertAndGrab,4 0.5 sec done", true));
//    	addSequential(new IntakeSetSpeed(RobotMap.intakePercentOut));
    	addSequential(new LogMessage("IntakeInvertAndGrab,5 intake out done", true));
    }
}