
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
	public static final int AUTO_COLS = 4;
	public enum AutoPath {
		SwitchPriority, BothSwitchesMiddle, ScaleOnly, ScalePriority
	}
	
	// Auto field layouts
	public static final int AUTO_FIELD_LAYOUTS = 4;	
	public enum AutoFieldLayout {
		LL, LR, RL, RR
		// first letter is closest switch, second is scale
	}
	
	// Columns in Array are in order of LL, LR, RL, RR
	public static int[][] startingLeftAutoPrograms = { { 3, 3, 1, 2},  // Program 1
			{ 1, 2, 1, 2},  // Program 2
			{ 5, 5, 5, 5},  // Program 3
			{ 3, 3, 1, 4}   // Program 4
	};
	
	public static int[][] startingMiddleAutoPrograms = { { 5, 5, 5, 5},  // Program 1
			{ 5, 5, 5, 5},  // Program 2
			{ 5, 5, 5, 5},  // Program 3
			{ 5, 5, 5, 5}   // Program 4
			
	};
	
	public static int[][] startingRightAutoPrograms = { { 2, 1, 3, 3},  // Program 1
			{ 2, 1, 2, 1},  // Program 2
			{ 5, 5, 5, 5},  // Program 3
			{ 4, 1, 3, 3}   // Program 4
			
	};
	 
}
