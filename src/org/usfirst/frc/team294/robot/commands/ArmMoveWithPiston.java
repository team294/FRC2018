package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmMoveWithPiston extends CommandGroup {
	double destAngle;
	boolean finalPistonPosition;
	double currentAngle = Robot.protoArmMotor.getArmDegrees();

	public ArmMoveWithPiston() {
		// destAngle = SmartDashboard.getNumber("Desired Arm Angle (Piston Version)",
		// 0);
		// requires(Robot.protoArmMotor);
		finalPistonPosition = true;
		double destAngle = Robot.protoArmMotor.getCurrentArmTarget();
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
