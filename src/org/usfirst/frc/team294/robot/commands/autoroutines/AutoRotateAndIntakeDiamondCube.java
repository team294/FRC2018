package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoRotateAndIntakeDiamondCube extends CommandGroup {

	/**
	 * If the cube is stuck in the intake in the diamond configuration,
	 * then this code tries to rotate and intake it once.  If that fails,
	 * then it tries a second time.
	 * If the cube is in the claw, then this sequence exits immediately.
	 */
	public AutoRotateAndIntakeDiamondCube() {
		
		// If we have the cube in the intake in diamond shape (not in claw), then try rotating the cube
		// 1.  Stop flywheels (this need to be done, otherwise the diamond sequence doesn't work).
    	addSequential(new ConditionalCommand(new StopIntakeAndClawAndClimb()) {
			protected boolean condition() {
				Robot.log.writeLogEcho("Auto diamond condition,intake photo switch," + Robot.intake.getPhotoSwitch()
									+ ",claw photo switch," + Robot.claw.getPhotoSwitch());
				return (!Robot.claw.getPhotoSwitch());
			}
		});

		// 2.  Wait for flywheels to stop
    	addSequential(new ConditionalCommand(new WaitCommand(0.1)) {
			protected boolean condition() {
				Robot.log.writeLogEcho("Auto diamond condition,intake photo switch," + Robot.intake.getPhotoSwitch()
									+ ",claw photo switch," + Robot.claw.getPhotoSwitch());
				return (!Robot.claw.getPhotoSwitch());
			}
		});
    	
		// 3.  Try rotating the cube
    	addSequential(new ConditionalCommand(new IntakeInvertAndGrab()) {
			protected boolean condition() {
				Robot.log.writeLogEcho("Auto diamond condition,intake photo switch," + Robot.intake.getPhotoSwitch()
									+ ",claw photo switch," + Robot.claw.getPhotoSwitch());
				return (!Robot.claw.getPhotoSwitch());
			}
		});
    	
    	// If we have the cube in the intake in diamond shape (not in claw), then try rotating the cube (again)
    	// 0.  Wait from last attempt
    	addSequential(new ConditionalCommand(new WaitCommand(0.1)) {
			protected boolean condition() {
				Robot.log.writeLogEcho("Auto diamond condition,intake photo switch," + Robot.intake.getPhotoSwitch()
									+ ",claw photo switch," + Robot.claw.getPhotoSwitch());
				return (!Robot.claw.getPhotoSwitch());
			}
		});
    	
		// 1.  Stop flywheels (this need to be done, otherwise the diamond sequence doesn't work).
    	addSequential(new ConditionalCommand(new StopIntakeAndClawAndClimb()) {
			protected boolean condition() {
				Robot.log.writeLogEcho("Auto diamond condition,intake photo switch," + Robot.intake.getPhotoSwitch()
									+ ",claw photo switch," + Robot.claw.getPhotoSwitch());
				return (!Robot.claw.getPhotoSwitch());
			}
		});
    	
		// 2.  Wait for flywheels to stop
    	addSequential(new ConditionalCommand(new WaitCommand(0.1)) {
			protected boolean condition() {
				Robot.log.writeLogEcho("Auto diamond condition,intake photo switch," + Robot.intake.getPhotoSwitch()
									+ ",claw photo switch," + Robot.claw.getPhotoSwitch());
				return (!Robot.claw.getPhotoSwitch());
			}
		});
    	
		// 3.  Try rotating the cube
    	addSequential(new ConditionalCommand(new IntakeInvertAndGrab()) {
			protected boolean condition() {
				Robot.log.writeLogEcho("Auto diamond condition,intake photo switch," + Robot.intake.getPhotoSwitch()
				+ ",claw photo switch," + Robot.claw.getPhotoSwitch());
				return (!Robot.claw.getPhotoSwitch());
			}
		});

	}
}
