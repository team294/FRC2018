package org.usfirst.frc.team294.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class OuttakeAll extends CommandGroup {

    public OuttakeAll() {

    	addParallel(new IntakeShootOut()); //need to write correct intake out sequence TODO
    	addSequential(new CubeShootOut()); 
    	
    }
}
