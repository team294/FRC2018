
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
	
	//Drive Train Constants
	public static final double wheelCircumference = 4.0 * Math.PI;
	public static final double encoderTicksPerRevolution = 4096.0;
	public static final double driveTrainDistanceFudgeFactor = 0.96824; //TODO: store in robot preferences
	
	// Pneumatic addresses
	public static int pnuematicShifterLow = 1;
	public static int pnuematicShifterHigh = 0;
	public static int pneumaticArmPistonIn = 3;
	public static int pneumaticArmPistonOut = 2;
	public static int pistonRetractedLimitSwitch = 0; // Are some of these analog ins for the limit switches? Need to separate
	
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
}
