
package org.usfirst.frc.team294.robot;

public class RobotMap {

	// Hardware CAN addresses
	public static final int leftMotor1 = 10;
	public static final int leftMotor2 = 11;
	public static final int leftMotor3 = 12;
	public static final int rightMotor1 = 20; 
	public static final int rightMotor2 = 21;
	public static final int rightMotor3 = 22;
	public static final int armMotor1 = 30;
	public static final int armMotor2 = 31;
	public static final int clawMotorLeft = 40;
	public static final int clawMotorRight = 41;
	public static final int intakeMotorLeft = 50;
	public static final int intakeMotorRight = 51;
	public static final int climbMotor1 = 60;
	public static final int climbMotor2 = 61;
	
	
	//Drive Train Constants
	public static final double wheelCircumference = 4.0 * Math.PI;
	public static final double encoderTicksPerRevolution = 4096.0;
	public static final double driveTrainDistanceFudgeFactor = 0.96824; //TODO: store in robot preferences
	
	// Pneumatic addresses
	public static final int pnuematicShifterLow = 1;
	public static final int pnuematicShifterHigh = 0;
	public static final int pneumaticArmPistonMajorIn = 3;
	public static final int pneumaticArmPistonMajorOut = 2;
	public static final int pneumaticClawPistonIn = 4;
	public static final int pneumaticClawPistonOut = 5;
	public static final int pneumaticIntakePistonIn = 6;
	public static final int pneumaticIntakePistonOut = 7;
	public static final int pneumaticArmPistonMinorIn = 8;
	public static final int pneumaticArmPistonMinorOut = 9;

	
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
	public static int[][] startingLeftAutoPrograms = { { 3, 3, 1, 2},  // Plan 0
			{ 3, 3, 1, 6},  // Plan 1
			{ 3, 3, 1, 4},  // Plan 2
			{ 1, 2, 1, 2},  // Plan 3
			{ 5, 5, 5, 5},  // Plan 4
			{ 7, 7, 7, 7}	// Plan 5
	};
	
	public static int[][] startingMiddleAutoPrograms = { { 5, 5, 5, 5},  // Plan 0
			{ 5, 5, 5, 5},  // Plan 1
			{ 5, 5, 5, 5},  // Plan 2
			{ 5, 5, 5, 5},  // Plan 3
			{ 5, 5, 5, 5},  // Plan 4
			{ 7, 7, 7, 7}   // Plan 5			
	};
	
	public static int[][] startingRightAutoPrograms = { { 2, 1, 3, 3},  // Plan 0
			{ 6, 1, 3, 3},  // Plan 1
			{ 4, 1, 3, 3},  // Plan 2
			{ 2, 1, 2, 1},  // Plan 3
			{ 5, 5, 5, 5},  // Plan 4
			{ 7, 7, 7, 7}	// Plan 5
			
	};
	
}
