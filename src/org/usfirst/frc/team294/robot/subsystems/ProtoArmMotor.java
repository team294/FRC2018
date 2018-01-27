package org.usfirst.frc.team294.robot.subsystems;


import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
 *
 */
public class ProtoArmMotor extends Subsystem {
	
	public final static WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.armMotor);
	
	public static int potValue = 0;
	public static Preferences robotPrefs;
	
	public static final double DEGREES_PER_CLICK = 0.5; //TODO: calibrate value
	
//	public static int countAtZeroDegrees;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	
	public ProtoArmMotor() {		
		robotPrefs = Preferences.getInstance();
		
		armMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
//		armMotor.set(ControlMode.Position, 3);
		armMotor.set(ControlMode.PercentOutput,0);
		armMotor.setNeutralMode(NeutralMode.Brake);
		armMotor.configForwardLimitSwitchSource( LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		armMotor.configReverseLimitSwitchSource( LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		armMotor.overrideLimitSwitchesEnable(true);  			//  pass false to force disable limit switch
		
	}
	
	
	
	public static void setArmMotorToPercentPower(double percent) {
		if (percent > .7) percent = .7;						//  Can be +/- 1  after testing
		if (percent < -.3) percent = -.3;
		if (percent < .1 && percent > -.1) percent = 0;
		armMotor.set(ControlMode.PercentOutput, percent);
		System.out.println("Arm motor " + armMotor.getDeviceID() + " set to percent " + percent + ", output " + armMotor.getMotorOutputVoltage() + " V," +
				armMotor.getOutputCurrent() + " A, Bus at " + armMotor.getBusVoltage() + " V");
		SmartDashboard.putNumber("Arm Motor Percent", percent);
	}
    
    public static void setArmPosition(double position) {
    	armMotor.set(ControlMode.Position, position);;
    	armMotor.config_kF(0, 0.0, 10);
        armMotor.config_kP(0, 0.1, 10);
        armMotor.config_kI(0, 0.0, 10);
        armMotor.config_kD(0, 0.0, 10);

    }

/** returns armPot corrected for zero degree reference
 * The reference value for the arm position at zero degrees is set in the Shuffleboard network table/preferences section
 * Set the arm to the zero position, set the countAtZeroDegrees to 0.  The value at that time read should then be entered into the countAtZeroDegree field.
 * The Arm Pot Value will now be calibrated and read 0
**/
    public int getArmPot () {				
    	potValue = armMotor.getSelectedSensorPosition(0)- robotPrefs.getInt("countAtZeroDegrees", 500);
    	SmartDashboard.putNumber("Arm Pot Value", potValue);
    	return (potValue); 
    }   
    
    public double getArmDegrees () {
    	double armAngle = getArmPot() * DEGREES_PER_CLICK;
    	SmartDashboard.putNumber("Arm angle Value", armAngle);
    	return(armAngle );
    }
    
    public void setArmAngle (double angle) {
    	double encoderDegrees = angle / DEGREES_PER_CLICK;
    	setArmPosition(encoderDegrees);
    }
    
    public void setArmFromSmartDashboard() {
    	double angle = SmartDashboard.getNumber("set Arm Angle", 0);
    	setArmAngle(angle);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
 //       setDefaultCommand(new ReadArmPotTesting());
    }
    
}

