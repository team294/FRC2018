package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath5_SwitchFromMiddle extends CommandGroup {

	/**
	 * Robot starts in the middle and puts cube on switch.
	 * @param goLeft true = put cube on left plate, false = put cube on right plate
	 */
    public AutoPath5_SwitchFromMiddle(boolean goLeft) {
		addSequential( new DriveStraightDistanceProfile(55, 0));
    	if(goLeft) {
    		addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
    		addSequential(new DriveStraightDistanceProfile(60, -90));
    		// arm code 
    	} 
    	else {
    		addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
    		addSequential(new DriveStraightDistanceProfile(50, 90));
    		//arm code
    	}
		addSequential( new TurnGyro( 0, TurnGyro.Units.Degrees));
		addSequential(new DriveStraightDistanceProfile(50, 0));
    }
}
