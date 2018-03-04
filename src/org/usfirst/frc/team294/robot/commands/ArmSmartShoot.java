package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

/**
 *
 */
public class ArmSmartShoot extends CommandGroup {

    public ArmSmartShoot() {
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
    	
    	addSequential(new ConditionalCommand(new OuttakeAll(), new ConditionalCommand(new ClawSetState(true), new CubeShootOut()) { 
			protected boolean condition() {
				return (RobotMap.getArmZone(Robot.armMotor.getArmDegrees()) == ArmZones.Middle);
			}} )	{
			protected boolean condition() {
				return (RobotMap.getArmZone(Robot.armMotor.getArmDegrees()) == ArmZones.Low);
			}
		});
    }
}
