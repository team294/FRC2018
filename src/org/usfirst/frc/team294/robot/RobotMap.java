
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
	public static int pnuematicShifterLow = 1;
	public static int pnuematicShifterHigh = 0;
	public static int pneumaticArmPistonIn = 3;
	public static int pneumaticArmPistonOut = 2;
	public static int pistonRetractedLimitSwitch = 0; // Are some of these analog ins for the limit switches? Need to separate
	
	// Arm angle constants
	public static double degreesToTicks = 0.5;
	public static double ticksToDegrees = 2.0; // Need to calibrate these values
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
}
