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

		addSequential(new ArmMoveToLegalRange());
		addParallel(new ArmMoveToEdge(destAngle));
		addSequential(new ArmPistonSmartRetract(destAngle, finalPistonPosition));
		addParallel(new ArmPistonSmartExtendInDestZone(destAngle));
		addSequential(new ArmMoveToDestAngle(destAngle));

	}
}
