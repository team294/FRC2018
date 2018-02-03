
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
	
	//Drive Train Constants
	public static final double wheelCircumference = 4.0 * Math.PI;
	public static final double encoderTicksPerRevolution = 4096.0;
	public static final double driveTrainDistanceFudgeFactor = 0.96824; //TODO: store in robot preferences
	
	// Pneumatic addresses
	public static final int pnuematicShifterLow = 1;
	public static final int pnuematicShifterHigh = 0;
	public static final int pneumaticArmPistonIn = 3;
	public static final int pneumaticArmPistonOut = 2;
	public static final int pistonRetractedLimitSwitch = 0; // Are some of these analog ins for the limit switches? Need to separate
	
	// Arm angle thresholds
	public static double lowThreshold; // Low threshold for ground pickup
	public static double highThreshold; // High threshold for scoring
	
	// Arm angle constants
	public static double degreesPerTicks = 9.0 / 5.0;
	public static double ticksPerDegrees = 5.0 / 9.0; // Need to calibrate these values
	public static double armLowPosition;
	public static double armHighPosition;
	
	public enum ArmPositions {
		Low, High, UltraHigh, WayTooHigh
	}
	
	public static double getArmAngle(ArmPositions position) {
		if (position == ArmPositions.Low) return armLowPosition;
		if (position == ArmPositions.High) return armHighPosition;
		else return armLowPosition;
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
