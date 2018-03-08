
package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;


/**
 * Sequence to load cube from intake to arm and reverse intake motors.
 * Load with intake closed.
 */
public class LoadCubeSequence extends CommandGroup {

    public LoadCubeSequence() {

    	/* These commands are all individual because we want them to finish before we continue on to moving anything else, to avoid impacts */
    	

    	addSequential(new IntakeSetDeploy(true)); // Deploy the intake first, before anything else
    	addSequential(new IntakeSetOpen(false));
    	addSequential(new ClawSetState(false)); // Close the claw while moving the arm
    	addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos,true)); // Move the arm to the intake position
    	// TODO Commented out the Wait command.  If intake is closed, we shouldn't need to wait.
    	//addSequential(new WaitCommand(.75));
    	addParallel(new IntakeCube()); // Open intake claw and start intaking, close when the photoswitch is triggered
    	addSequential(new ArmIntakeCube()); // Simultaneously, open the arm claw and being intaking. Exit when bumpswitch triggered.
    	addSequential(new PassiveOuttake()); // Start outtaking so we don't get a penalty
		addSequential(new PassiveClawIntake()); // intake so it doesn't drop cube
	}
}
