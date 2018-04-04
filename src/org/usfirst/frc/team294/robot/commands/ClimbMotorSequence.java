package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class ClimbMotorSequence extends CommandGroup {

    public ClimbMotorSequence() {
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
 
    	addParallel(new ArmMoveWithPiston(RobotMap.armIntakePos, false));
    	addParallel(new ClimbPistonDeploy(false)); //setting up 
    	addSequential(new WaitCommand(0.5));
    	addParallel(new ClimbSetPercentPower(RobotMap.climbPercent));
    	addSequential(new WaitCommand(4.0));			//  test this time or add sensor when over 12 inches
    	addSequential(new ClimbSetPercentPower(-0.3));
    	addSequential(new WaitCommand(26.0));
    	addSequential(new ClimbSetPercentPower(0.0));
    }
}
