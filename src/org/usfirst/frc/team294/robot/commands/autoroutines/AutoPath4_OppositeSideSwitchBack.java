package org.usfirst.frc.team294.robot.commands.autoroutines;

import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.TurnGyro;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutoPath4_OppositeSideSwitchBack extends CommandGroup {

    public AutoPath4_OppositeSideSwitchBack(int startPosition) {
    	switch (startPosition) {
		case 1:
			addSequential(new DriveStraightDistanceProfile(-230, 0));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			if(true||false) {//because java is dumb
				return; //Dont delete, unless the beam in the middle of the test field doesnt exist
			}
			addSequential(new DriveStraightDistanceProfile(-200, 90));
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(50, 0));
			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(20, 90)); // Touch the Switch wall
			// Score cube

// Reference code
//			addSequential(new DriveStraightDistanceProfile(217, 0));
//			addSequential(new TurnGyro(90, TurnGyro.Units.Degrees));
//			if(true||false) {//because java is dumb
//			return; //Dont delete, unless the beam in the middle of the test field doesnt exist
//			}
//			addSequential(new DriveStraightDistanceProfile(200, 90));
//			addSequential(new TurnGyro(180, TurnGyro.Units.Degrees));
//			addSequential(new DriveStraightDistanceProfile(50, 180));
//			addSequential(new TurnGyro(270, TurnGyro.Units.Degrees));
			break;
		case 3:
			addSequential(new DriveStraightDistanceProfile(-230, 0));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(-152, -90)); // Actually go 165" but also shortchange because beam
			addSequential(new TurnGyro(0, TurnGyro.Units.Degrees));
			if(true||false) {//because java is dumb
				return; //Dont delete, unless the beam in the middle of the test field doesnt exist
			}
			addSequential(new DriveStraightDistanceProfile(50, 0));
			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
			addSequential(new DriveStraightDistanceProfile(20, -90)); // Touch the Switch wall
			// Score cube
			
//			addSequential(new DriveStraightDistanceProfile(210, 0)); //Actually go 230"
//			addSequential(new TurnGyro(-90, TurnGyro.Units.Degrees));
//			addSequential( new DriveStraightDistanceProfile(152, -90)); // Actually go 165" but also shortchange because beam
//			addSequential(new TurnGyro(180, TurnGyro.Units.Degrees));
//			if(true||false) {//because java is dumb
//				return; //Dont delete, unless the beam in the middle of the test field doesnt exist
//			}
//			addSequential( new DriveStraightDistanceProfile(50, -180));
//			addSequential(new TurnGyro(-270, TurnGyro.Units.Degrees));
//			// put arm command here 
//			break;

		}

    }
}
