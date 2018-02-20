package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmPositions;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class ArmMoveWithPiston extends CommandGroup {
	double destAngle;
	boolean finalPistonPosition;
	double currentAngle = Robot.armMotor.getArmDegrees();

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
		addSequential(new LogMessage("Arm Move started, angle = " + destAngle + ", finalPistonPosition = " + finalPistonPosition, true));
		addSequential(new ArmMoveToLegalRange());
		addSequential(new LogMessage("Arm Move: in legal range", true));
		addParallel(new ArmMoveToEdge(destAngle));
		addSequential(new ArmPistonSmartRetract(destAngle, finalPistonPosition));
		addSequential(new LogMessage("Arm Move: smart retract done", true));
		addParallel(new ArmPistonSmartExtendInDestZone(destAngle));
		addSequential(new ArmMoveToDestAngle(destAngle));
		addSequential(new LogMessage("Arm Move: at dest angle", true));
		
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
	
	/**
	 * Moves the arm and adjusts the piston in/out as needed to stay in legal
	 * volume.
	 * @param position desired angle, from RobotMap.ArmPositions
	 * @param finalPistonPosition true = extend piston by end of arm movement, false = retract
	 */
	public ArmMoveWithPiston(ArmPositions position, boolean finalPistonPosition) {
		this(position.getAngle(), finalPistonPosition);
	}
	
	/**
	 * Moves the arm and adjusts the piston in/out as needed to stay in legal
	 * volume.
	 * @param position desired angle, from RobotMap.ArmPositions
	 */
	public ArmMoveWithPiston(ArmPositions position) {
		this(position.getAngle(), true);
	}
}
