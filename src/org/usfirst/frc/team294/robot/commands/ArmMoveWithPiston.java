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

	/**
	 * Moves the arm and adjusts the piston in/out as needed to stay in legal
	 * volume.
	 * 
	 * @param destAngle
	 *            Desired destination angle, in degrees. 0 is horizontal, + is up, -
	 *            is down.
	 * @param finalPistonPosition
	 *            true = extend piston by end of arm movement, false = retract
	 *            piston
	 */
	public ArmMoveWithPiston(double destAngle, boolean finalPistonPosition) {
		// destAngle = SmartDashboard.getNumber("Desired Arm Angle (Piston Version)",
		// 0);
		// requires(Robot.protoArmMotor);

		addSequential(new ArmMoveToLegalRange());
		addParallel(new ArmPistonSmartRetract(destAngle, finalPistonPosition));
		addSequential(new ArmMoveToEdge(destAngle));
		addSequential(new ArmMoveToDestAngle(destAngle));
		// if(finalPistonPosition) {
		addSequential(new ArmPistonSmartExtendInDestZone(destAngle));
		// }

		/*
		 * if (RobotMap.getArmZone(currentAngle) == RobotMap.getArmZone(destAngle)) {
		 * addParallel(new ArmMoveToDestAngle(destAngle)); if (finalPistonPosition) {
		 * addSequential(new ArmPistonSmartExtend()); } } else { addParallel(new
		 * ArmMoveToEdge(destAngle)); addSequential(new ArmPistonRetractBoth());
		 * addParallel(new ArmMoveToDestAngle(destAngle)); addSequential(new
		 * ArmPistonSmartExtendInDestZone(destAngle));
		 * 
		 * }
		 */

	}
}