package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;

/**
 *
 */
public class SmartShoot extends CommandGroup {

	ArmZones currentZone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
	boolean intakePosition = Robot.intake.intakeDeployed();
	
	
	
    public SmartShoot() {
    	
    	addSequential(new ConditionalCommand(new ArmSmartShoot(), new ConditionalCommand(new IntakeShootOut(), new OuttakeAll()) { 
			protected boolean condition() {
				return Robot.intake.isCubeInIntake();
			}} )	{
			protected boolean condition() {
				return Robot.claw.getPhotoSwitch();
			}
		});
    	
    }
}
