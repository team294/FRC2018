package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *Sequence to move the intake out of the way and bring the arm to the switch position
 */
public class ArmMoveWithIntake extends CommandGroup {

	public ArmMoveWithIntake() {
		addSequential(new IntakeSetDeploy(true));
		addSequential(new WaitCommand(0.25));
		addParallel(new ArmMoveWithPiston(RobotMap.armSwitchPosHigh, false));
		addSequential(new WaitCommand(1.0));
		addSequential(new IntakeSetDeploy(false));
	}
}
