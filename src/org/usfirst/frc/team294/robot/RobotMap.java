
package org.usfirst.frc.team294.robot;

public class RobotMap {

	// Hardware CAN addresses
	public static final int leftMotor1 = 10;
	public static final int leftMotor2 = 11;  	// Master
	public static final int leftMotor3 = 12;
	public static final int rightMotor1 = 20; 
	public static final int rightMotor2 = 21;	// Master
	public static final int rightMotor3 = 22;
	public static final int armMotor1 = 30;
	public static final int armMotor2 = 31;
	public static final int clawMotorLeft = 40;
	public static final int clawMotorRight = 41;
	public static final int intakeMotorLeft = 50;
	public static final int intakeMotorRight = 51;
	public static final int climbMotor1 = 60;
	public static final int climbMotor2 = 61;

	
	
	// Pneumatic addresses
	
	public static final int pnuematicShifterHigh = 0;
	public static final int pnuematicShifterLow = 1;		// this is used for prototype drive base only!!  Change to single solenoid in prototype
	public static final int pnuematicArmBrake = 1;		
	public static final int pneumaticArmPistonMajorOut = 2;
	public static final int pneumaticArmPistonMajorIn = 3;
	public static final int pneumaticIntakePistonGrab = 4;
	public static final int pneumaticClawPistonOut = 5;
	public static final int pneumaticArmPistonMinorOut = 6;
	public static final int pneumaticIntakePistonDeploy = 7;
	
	
	// RoboRIO digital I/O addresses
	public static final int majorPistonRetractedLimitSwitch = 0; 
	public static final int majorPistonExtendedLimitSwitch = 1; 
	public static int photoSwitchIntake = 2;
	public static int photoSwitchClaw = 3;
	public static int bumpSwitchClaw = 4;
	public static final int minorPistonRetractedLimitSwitch = 5; 
	public static final int minorPistonExtendedLimitSwitch = 6; 

	//intake motor speeds
	public static double intakePercentIn = .7; //need to be tested
	public static double intakePercentOut = -0.3;
	
	//claw motor speeds
	public static double clawPercentIn = .7; //need to be tested
	public static double clawPercentOut = -0.3;
	
	// Arm angle thresholds
	public static double lowThreshold; // Low threshold for ground pickup
	public static double highThreshold; // High threshold for scoring

		
		//Drive Train Constants
		public static final double wheelCircumference = 4.0 * Math.PI;
		public static final double encoderTicksPerRevolution = 4096.0;
		public static final double driveTrainDistanceFudgeFactor = 0.96824; //TODO: store in robot preferences
		
	

	
	// Arm angle constants
	public static double degreesPerTicks = 9.0 / 5.0;
	public static double armLowPosition;
	public static double armHighPosition;
	
	
	public enum ArmPositions {
		Low, High, UltraHigh, WayTooHigh
	} // Enum for preset positions to use in the code (e.g. placing on scale low, switch, pickup, etc.
	
	
	// Arm angle zone boundaries   THESE ARE NUMBERS JUST FOR TEST ON PROTO ARM
	public static double minAngle = -37; // arm cannot extend downward past this angle
	public static double lowerBound = -25; // piston1 can be extended between Ang0 and Ang1, cube picked up below Ang1
	public static double middleBound = 30; // arm cannot extend between Ang1 and Ang2
	public static double upperBound = 130; // both pistons can be extended between Ang2 and Ang 3
	public static double maxAngle = 150; // arm cannot extend upward past this angle
	
	public enum ArmZones {
		Low, Middle, High, Backwards
	}

	// arm angles where claw pistons cannot be opened
	public static double angleClawCloseLow = -20;
	public static double angleClawCloseHigh = -10;
	
	/*****************************************/
	//   WHY IS THIS IN MAP??  SHOULDN'T BE IN THE SUBSYSTEM?
	/**
	 * Returns the zone of the arm based on the arm angle
	 * @param angle
	 * @return RobotMap.ArmZones
	 */
	public static ArmZones getArmZone(double angle) {
		if (angle < lowerBound) return ArmZones.Low;
		if (angle >= upperBound) return ArmZones.Backwards;
		if (angle > middleBound) return ArmZones.High;
		return ArmZones.Middle;
	}
	public static double getArmAngle(ArmPositions position) {
		if (position == ArmPositions.Low) return armLowPosition;
		if (position == ArmPositions.High) return armHighPosition;
		else return armLowPosition;
	}
/***********************************************/
	
	public enum PistonPositions {
		Extended, Retracted, Moving, Null
	}

	// Auto path selections
	public static final int AUTO_PLANS = 5;
	public enum AutoPlan {
		ClosestSwitchScale_FFScale, ClosestSwitchScale_FFSwitchFront, ClosestSwitchScale_FFSwitchBack, ScaleOnly, SwitchOnly 
	}
	
	// Auto field layouts
	public static final int AUTO_FIELD_LAYOUTS = 4;	
	public enum AutoFieldLayout {
		LL, LR, RL, RR
		// first letter is closest switch, second is scale
	}
	
	// Starting positions
	public enum StartingPosition {
		Left, Middle, Right
	}
	
	// Columns in Array are in order of LL, LR, RL, RR
	public static int[][] startingLeftAutoPrograms = { { 3, 3, 1, 2},  // Plan 1
			{ 3, 3, 1, 6},  // Plan 2
			{ 3, 3, 1, 4},  // Plan 3
			{ 1, 2, 1, 2},  // Plan 4
			{ 5, 5, 5, 5}   // Plan 5
	};
	
	public static int[][] startingMiddleAutoPrograms = { { 5, 5, 5, 5},  // Plan 1
			{ 5, 5, 5, 5},  // Plan 2
			{ 5, 5, 5, 5},  // Plan 3
			{ 5, 5, 5, 5},  // Plan 4
			{ 5, 5, 5, 5}   // Plan 5			
	};
	
	public static int[][] startingRightAutoPrograms = { { 2, 1, 3, 3},  // Plan 1
			{ 6, 1, 3, 3},  // Plan 2
			{ 4, 1, 3, 3},  // Plan 3
			{ 2, 1, 2, 1},  // Plan 4
			{ 5, 5, 5, 5}   // Plan 5
			
	};
	
}
