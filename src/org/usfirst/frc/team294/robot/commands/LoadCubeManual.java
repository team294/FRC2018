package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class LoadCubeManual extends CommandGroup {

    public LoadCubeManual() {
    	
    	addSequential(new IntakeSetDeploy(true)); // Deploy the intake first, before anything else
    	addSequential(new IntakeSetOpen(false));// Close the intake while moving the arm
    	addSequential(new ClawSetOpen(true));
    	addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos, false)); // Close the claw while moving the arm
    	
    	// TODO Commented out the Wait command.  If intake is closed, we shouldn't need to wait.
    	//addSequential(new WaitCommand(.75));
    	addParallel(new IntakeCube()); // Open intake claw and start intaking, close when the photoswitch is triggered
    	addSequential(new ClawSetMotorSpeed(RobotMap.clawPercentIn));
    }
}
