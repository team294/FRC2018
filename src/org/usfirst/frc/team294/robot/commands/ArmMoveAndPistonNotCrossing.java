package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmMoveAndPistonNotCrossing extends CommandGroup {
	double destAngle;
	boolean finalPistonPosition;
	double currentAngle = Robot.protoArmMotor.getArmDegrees();

	public ArmMoveAndPistonNotCrossing(double destAngle, boolean finalPistonPosition) {
		// destAngle = SmartDashboard.getNumber("Desired Arm Angle (Piston Version)",
		// 0);
		// requires(Robot.protoArmMotor);
		addSequential(new ArmMoveToLegalRange());
		if (RobotMap.getArmZone(currentAngle) == RobotMap.getArmZone(destAngle)) {
			addParallel(new ArmMoveToDestAngle(destAngle));
			if (finalPistonPosition) {
				addSequential(new ArmPistonSmartExtend());
			}
		} else {
			addParallel(new destAngle());
			addSequential(new ArmPistonRetractBoth());
			addParallel(new ArmMoveToDestAngle(destAngle));
			addSequential(new ArmPistonSmartExtendInDestZone(destAngle));

		}
	}
}
