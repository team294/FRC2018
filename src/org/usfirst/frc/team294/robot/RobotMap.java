
package org.usfirst.frc.team294.robot;

public class RobotMap {
	public static final int leftMotor1 = 10;
	public static final int leftMotor2 = 11;
	public static final int leftMotor3 = 12;
	public static final int rightMotor1 = 20;
	public static final int rightMotor2 = 21;
	public static final int rightMotor3 = 22;
	public static final int pnuematicShifterLow = 1;
	public static final int pnuematicShifterHigh = 0;
	public static final double wheelCircumference = 4.0 * Math.PI;
	public static final double encoderTicksPerRevolution = 4096.0;
	public static final double driveTrainDistanceFudgeFactor = 0.96824; //TODO: store in robot preferences
}
