
package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;


/**
 * Sequence to load cube from intake to arm and reverse intake motors.
 * Load with intake closed.
 */
public class LoadCubeSequenceNoIntake extends CommandGroup {

    public LoadCubeSequenceNoIntake() {

    	addSequential(new ArmMoveWithPiston(RobotMap.armIntakePos, PistonPositions.Extended));
    	addSequential(new ArmPistonSmartExtend());
    	addSequential(new ArmIntakeCube()); 
    	addSequential(new ArmPistonRetract(true));
    	addSequential(new ArmMoveWithPiston(-53, false));
   	}
}
