
package org.usfirst.frc.team294.robot;

public class RobotMap {
	public static final int leftMotor1 = 10;
	public static final int leftMotor2 = 11;
	public static final int leftMotor3 = 12;
	public static final int rightMotor1 = 20; 
	public static final int rightMotor2 = 21;
	public static final int rightMotor3 = 22;
	public static final int pnuematicShifter = 2;
	
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
	
}
