package org.usfirst.frc.team294.robot.subsystems;


import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.ReadArmPotTesting;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



/**
 *
 */
public class ProtoArmMotor extends Subsystem {
	
	public final static WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.armMotor);
	
	int POT_ZERO_REFERENCE = 500; //  TODO add method to Calibrate andstore this in Network Nables and read from NT
	public static int potValue = 0;
	public static int countAtZeroDegrees;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	
	public ProtoArmMotor() {		
		armMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
		armMotor.set(ControlMode.Position, 3);
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
		
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new ReadArmPotTesting());
    }
    
    
    public static int readArmPot () {				// returns armPot corrected for zero degree reference
    	potValue = armMotor.getSelectedSensorPosition(0); // - countAtZeroDegrees;
    	SmartDashboard.putNumber("Arm Pot Value", potValue);
    	return (potValue);
    }
}

