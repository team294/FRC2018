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
