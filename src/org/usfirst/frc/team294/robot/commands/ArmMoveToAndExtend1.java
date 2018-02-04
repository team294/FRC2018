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
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	requires(Robot.protoArmMotor);
    	addSequential(new ArmMoveToLegalRange());
    	if (RobotMap.getArmZone(currentAngle) == RobotMap.getArmZone(destAngle)) {
    		addParallel(new ArmMoveToDestAngle(destAngle));
    		if(finalPistonPosition) {
    		addSequential(new ArmPistonSmartExtend());
    		}
    		}
    	else {
    		addParallel(new ArmMoveToEdge(destAngle));
    		//addSequential();
    		
    		
    	}
    }
}
