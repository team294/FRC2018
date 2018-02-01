
package org.usfirst.frc.team294.robot;

public class RobotMap {
	
	// Hardware CAN addresses
	public static int leftMotor1 = 10;
	public static int leftMotor2 = 11;
	public static int leftMotor3 = 12;
	public static int rightMotor1 = 20; 
	public static int rightMotor2 = 21;
	public static int rightMotor3 = 22;
	public static int armMotor = 30;
	
	// Pneumatic addresses
	public static int pnuematicShifterHigh = 0;
	public static int pnuematicShifterLow = 1;
	public static int pneumaticArmPistonMajorOut = 2;
	public static int pneumaticArmPistonMajorIn = 3;
	public static int pneumaticArmPistonMinorOut = 4;
	public static int pneumaticArmPistonMinorIn = 5;

	// Digital Input addresses
	public static int majorPistonRetractedLimitSwitch = 0;
	public static int majorPistonExtendedLimitSwitch = 1;
	public static int minorPistonRetractedLimitSwitch = 2;
	public static int minorPistonExtendedLimitSwitch = 3;
	
	// Arm angle thresholds
	public static double lowThreshold; // Low threshold for ground pickup
	public static double highThreshold; // High threshold for scoring
	
	// Arm angle constants
	public static double degreesPerTicks = 9.0 / 5.0;
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
	
	public enum PistonPositions {
		Extended, Retracted, Moving, Null
	}
}
