
package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;


/**
 * Sequence to load cube from intake to arm and reverse intake motors
 */
public class LoadCubeSequence extends CommandGroup {

    public LoadCubeSequence() {
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
    	
    	/* These commands are all individual because we want them to finish before we continue on to moving anything else, to avoid impacts */
    	
    	if (!Robot.claw.getPhotoSwitch()) {
    		addSequential(new IntakeSetDeploy(true)); // Deploy the intake first, before anything else
    		addSequential(new IntakeSetOpen(true));
    		addSequential(new ClawSetState(false)); // Close the claw while moving the arm
    		addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos,true)); // Move the arm to the intake position
//    		TODO add checker to see if the intake is deployed and open before intaking can occur
    		addSequential(new WaitCommand(.75));
    		addParallel(new IntakeCube()); // Open intake claw and start intaking, close when the photoswitch is triggered
    		addSequential(new ArmIntakeCube()); // Simultaneously, open the arm claw and being intaking. Exit when bumpswitch triggered.
    		addSequential(new PassiveOuttake()); // Start outtaking so we don't get a penalty
    	}
		addSequential(new PassiveClawIntake()); // intake so it doesn't drop cube
	}
}
