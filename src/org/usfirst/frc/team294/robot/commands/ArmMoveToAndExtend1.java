package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmMoveToAndExtend1 extends CommandGroup {
	double destAngle;
	boolean finalPistonPosition;
	double currentAngle = Robot.protoArmMotor.getArmDegrees();

	public ArmMoveToAndExtend1(double destAngle, boolean finalPistonPosition) {
		requires(Robot.protoArmMotor);
		addSequential(new ArmMoveToLegalRange());
		if (RobotMap.getArmZone(currentAngle) == RobotMap.getArmZone(destAngle)) {
			addParallel(new ArmMoveToDestAngle(destAngle));
			if (finalPistonPosition) {
				addSequential(new ArmPistonSmartExtend());
			}
		} else {
			addParallel(new ArmMoveToEdge(destAngle));
			addSequential(new ArmPistonRetractBoth());
			addParallel(new ArmMoveToDestAngle(destAngle));
			addSequential(new ArmPistonSmartExtendInDestZone(destAngle));

		}
	}
}
