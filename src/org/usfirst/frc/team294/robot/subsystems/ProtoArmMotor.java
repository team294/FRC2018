package org.usfirst.frc.team294.robot.subsystems;


import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.UpdateArmSmartDashboard;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
**/


/**
 * The subsystem controlling the arm angle (but not the piston)
 */
public class ProtoArmMotor extends Subsystem {
	
	private final WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.armMotor);
		
	private double DEGREES_PER_TICK = RobotMap.degreesPerTicks;
	private double TICKS_PER_DEGREE = RobotMap.ticksPerDegrees;
	private double armZeroDegreesCalibration = Robot.armCalZero;
	private double arm90DegreesCalibration = Robot.armCal90Deg;
	
	//TODO: Add code for arm angles from preferences in Robot.java
	
	public ProtoArmMotor() {		
		
		//armMotor.set(ControlMode.Position, 3);
		armMotor.set(ControlMode.PercentOutput,0);
		armMotor.setNeutralMode(NeutralMode.Brake);
		armMotor.configForwardLimitSwitchSource( LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		armMotor.configReverseLimitSwitchSource( LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		armMotor.overrideLimitSwitchesEnable(true);  			//  pass false to force disable limit switch
		
		// Closed-loop control structures
		armMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
		armMotor.setSensorPhase(true);
		//armMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 0, 0x00, 0x00, 0x00); // Change parameter to 1 for non-continuous
		armMotor.selectProfileSlot(0, 0);
		armMotor.config_kF(0, 0.0, 10);
        armMotor.config_kP(0, 50.0, 10);
        armMotor.config_kI(0, 0.0, 10);
        armMotor.config_kD(0, 0.0, 10);
        armMotor.configClosedloopRamp(0.25, 10);
        armMotor.configPeakOutputForward(0.8, 10);
        armMotor.configPeakOutputReverse(-0.5, 10);
	}
	
	public void calibrate() {
		//yeah we're gonna fill this out later with uhhhhh something
	}
	
	
	/**
	 * Controls the arm based on Percent VBUS
	 * @param percent voltage, minimum -0.3 and maximum 0.7
	 */
	public void setArmMotorToPercentPower(double percent) {
		if (percent > .7) percent = .7;						//  Can be +/- 1  after testing
		if (percent < -.3) percent = -.3;
		if (percent < .1 && percent > -.1) percent = 0;
		armMotor.set(ControlMode.PercentOutput, percent);
		System.out.println("Arm motor " + armMotor.getDeviceID() + " set to percent " + percent + ", output " + armMotor.getMotorOutputVoltage() + " V," +
				armMotor.getOutputCurrent() + " A, Bus at " + armMotor.getBusVoltage() + " V");
		SmartDashboard.putNumber("Arm Motor Percent", percent);
	}

	/** 
	 * Returns value of pot on arm, adjusted for zero degree reference at level. Also updates Smart Dashboard. </br>
	 * The reference value for the arm position at zero degrees is set in the Shuffleboard network table/preferences section.</br>
	 * <b>To reset:</b> Set the arm to the zero position, set the countAtZeroDegrees to 0.
	 * The value at that read should then be entered into the countAtZeroDegree field.
	**/
    public double getArmPot() {				
    	double potValue = armMotor.getSelectedSensorPosition(0) - (-245);//armZeroDegreesCalibration; //Removed zero for testing purposes
    	//int potValue = armMotor.getSensorCollection().getAnalogIn();
    	SmartDashboard.putNumber("Arm Pot Value", potValue);
    	return (potValue); 
    }
    
    /**
     * Returns the raw value of the pot on arm, without adjusting for level. Also updates SmartDashboard.
     * @return raw value, probably a large negative number
     */
    public double getArmPotRaw() {
    	double potVal = armMotor.getSelectedSensorPosition(0);
    	SmartDashboard.putNumber("Arm Pot Raw", potVal);
    	return potVal;
    }
    
    /**
     * Gets the current angle of the arm, in degrees (converted from potentiometer)
     * </br>Uses getArmPot and multiplies by degrees per click constant
     * @return angle in degrees
     */
    public double getArmDegrees() {
    	double armAngle = getArmPot() * DEGREES_PER_TICK;
    	SmartDashboard.putNumber("Arm angle Value", armAngle);
    	return (armAngle);
    }
    
    /**
     * Sets the angle of the arm in degrees
     * @param angle desired angle, in degrees
     */
    public void setArmAngle(double angle) {
    	// TODO: add checks to make sure arm does not go outside of safe areas
    	// TODO: integrate checks with piston to avoid penalties for breaking frame perimeter
    	double encoderDegrees = angle * TICKS_PER_DEGREE;
    	setArmPosition(encoderDegrees);
    }
    
	/**
	 * Sets the position of the arm based on potentiometer ticks
	 * @param position desired position, in pot ticks
	 */
    private void setArmPosition(double position) {
    	position += (-245); //armZeroDegreesCalibration;
    	armMotor.set(ControlMode.Position, position);
    }
    
    /**
     * Sets the position of the arm in degrees according to the smart dashboard
     */
    public void setArmFromSmartDashboard() {
    	double angle = SmartDashboard.getNumber("set Arm Angle", 0);
    	setArmAngle(angle); // For closed-loop testing purposes, passing in encoder values instead of angles
    }
    
    /**
     * Gets the output voltage of the arm motor.</br>
     * Also updates smart dashboard with voltage
     * @return voltage
     */
    public double getOutputVoltage() {
    	double voltage = armMotor.getMotorOutputVoltage();
    	SmartDashboard.putNumber("Arm Motor Output Voltage", voltage);
    	return voltage;
    }
    
    /**
     * Updates pot and angle measurements on the SmartDashboard
     */
    public void updateSmartDashboard() {
    	getArmPot();
    	getArmDegrees();
    	getOutputVoltage();
    	SmartDashboard.putNumber("Arm Motor Error", armMotor.getClosedLoopError(0));
    	SmartDashboard.putNumber("Arm Motor Target", armMotor.getClosedLoopTarget(0));
    }
    
    public void periodic() {
    	updateSmartDashboard();
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
//    	setDefaultCommand(new UpdateArmSmartDashboard());
    }
    
    
    
}