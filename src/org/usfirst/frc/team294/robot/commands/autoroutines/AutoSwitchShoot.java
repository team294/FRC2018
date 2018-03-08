package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.ClawSetMotorSpeed;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoSwitchShoot extends CommandGroup {

    public AutoSwitchShoot() {
    	addSequential(new ClawSetMotorSpeed(.75));
		addSequential(new WaitCommand(.1));
		addSequential(new ClawSetMotorSpeed(.4));
		addSequential(new WaitCommand(1));
		addSequential(new ClawSetMotorSpeed(0));
    }
}
