
package org.usfirst.frc.team294.robot;

public class RobotMap {

	// Hardware CAN addresses
	public static final int leftMotor1 = 10;
	public static final int leftMotor2 = 11;
	public static final int leftMotor3 = 12;
	public static final int rightMotor1 = 20; 
	public static final int rightMotor2 = 21;
	public static final int rightMotor3 = 22;
	public static final int armMotor = 30;
	public static final int clawMotorLeft = 40;
	public static final int clawMotorRight = 41;

	
	
	// Pneumatic addresses
	public static int pnuematicShifterHigh = 0;
	public static int pnuematicShifterLow = 1;
	public static int pneumaticArmPistonMajorOut = 2;
	public static int pneumaticArmPistonMajorIn = 3;
	public static int pneumaticArmPistonMinorOut = 6;
	public static int pneumaticArmPistonMinorIn = 7;
	public static final int pneumaticClawPistonIn = 4;
	public static final int pneumaticClawPistonOut = 5;


	// Digital Input addresses
	public static int majorPistonRetractedLimitSwitch = 0;
	public static int majorPistonExtendedLimitSwitch = 1;
	public static int minorPistonRetractedLimitSwitch = 2;
	public static int minorPistonExtendedLimitSwitch = 3;
	public static int bumpSwitch = 4;	
	public static int photoSwitch = 5;

	
	// Arm angle thresholds
	public static double lowThreshold; // Low threshold for ground pickup
	public static double highThreshold; // High threshold for scoring

	
	
		
		//Drive Train Constants
		public static final double wheelCircumference = 4.0 * Math.PI;
		public static final double encoderTicksPerRevolution = 4096.0;
		public static final double driveTrainDistanceFudgeFactor = 0.96824; //TODO: store in robot preferences
		
	
		
		// RoboRIO digital I/O addresses
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	// Arm angle constants
	public static double degreesPerTicks = 9.0 / 5.0;
	public static double armLowPosition;
	public static double armHighPosition;
	
	
	public enum ArmPositions {
		Low, High, UltraHigh, WayTooHigh
	} // Enum for preset positions to use in the code (e.g. placing on scale low, switch, pickup, etc.
	
	public static double getArmAngle(ArmPositions position) {
		if (position == ArmPositions.Low) return armLowPosition;
		if (position == ArmPositions.High) return armHighPosition;
		else return armLowPosition;
	}
	
	// Arm angle zone boundaries
	public static double minAngle = -37; // arm cannot extend downward past this angle
	public static double lowerBound = -25; // piston1 can be extended between Ang0 and Ang1, cube picked up below Ang1
	public static double middleBound = 30; // arm cannot extend between Ang1 and Ang2
	public static double upperBound = 100; // both pistons can be extended between Ang2 and Ang 3
	public static double maxAngle = 130; // arm cannot extend upward past this angle
	
	public enum ArmZones {
		Low, Middle, High, Backwards
	}
	
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
