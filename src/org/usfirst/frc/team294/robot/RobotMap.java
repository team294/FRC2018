
package org.usfirst.frc.team294.robot;

public class RobotMap {
	public static final int leftMotor1 = 10;
	public static final int leftMotor2 = 11;
	public static final int leftMotor3 = 12;
	public static final int rightMotor1 = 20; 
	public static final int rightMotor2 = 21;
	public static final int rightMotor3 = 22;
	public static final int pnuematicShifter = 2;
	public static final int pnuematicShifterLow = 1;
	public static final int pnuematicShifterHigh = 0;
	
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
