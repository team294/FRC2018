package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
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
	 * @param finalPistonPosition (must be a constant)
	 *            true = extend piston by end of arm movement, false = retract
	 *            piston
	 */
	public ArmMoveWithPiston(double destAngle, boolean finalPistonPosition) {
		addSequential(new LogMessage("Arm Move,1,started,dest angle," + destAngle + ",finalPistonPosition," + finalPistonPosition, true));
		
		addSequential(new ArmMoveToLegalRange());
		addSequential(new LogMessage("Arm Move,2,in legal range", true));
		
		addParallel(new ArmMoveToEdge(destAngle));
		addSequential(new ArmPistonSmartRetract(destAngle, finalPistonPosition));
		addSequential(new LogMessage("Arm Move,3,smart retract done", true));

//		addParallel(new ArmPistonSmartRetract(destAngle, finalPistonPosition));   // For now, just tell the piston to retract
		
		if (finalPistonPosition) addParallel(new ArmPistonSmartExtendInDestZone(destAngle));
		addSequential(new ArmMoveToDestAngle(destAngle));
		addSequential(new LogMessage("Arm Move,4,at dest angle", true));
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
		this(position.getAngle(), false);
	}
}
