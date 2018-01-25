package org.usfirst.frc.team294.robot.subsystems;


import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
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
	
	public final WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.armMotor);
	
	int POT_ZERO_REFERENCE = 500; //  TODO add method to Calibrate andstore this in Network Nables and read from NT
	public static int PotValue=0;
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

	
	public ProtoArmMotor() {
		armMotor.set(ControlMode.PercentOutput, 0.0);
		armMotor.setNeutralMode(NeutralMode.Brake);
		armMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
		armMotor.configForwardLimitSwitchSource( LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		armMotor.configReverseLimitSwitchSource( LimitSwitchSource.FeedbackConnector,LimitSwitchNormal.NormallyOpen,0);
		armMotor.overrideLimitSwitchesEnable(true);  			//  pass false to force disable limit switch
		
	}
	
	
	
	public void setArmMotorToPercentPower(double percent) {
		if (percent > .3) percent = .3;						//  Can be +/- 1  after testing
		if (percent < -.3) percent = -.3;
		armMotor.set(ControlMode.PercentOutput, percent);
		System.out.println("Arm motor " + armMotor.getDeviceID() + " set to percent " + percent + ", output " + armMotor.getMotorOutputVoltage() + " V," +
				armMotor.getOutputCurrent() + " A, Bus at " + armMotor.getBusVoltage() + " V");
		SmartDashboard.putNumber("Arm Motor Percent", percent);
	}
		
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    
    public int readArmPot () {				// returns armPot corrected for zero degree reference
    	PotValue = armMotor.getSelectedSensorPosition(0)- POT_ZERO_REFERENCE;
    	SmartDashboard.putNumber("Arm Pot Value", PotValue);
    	return (PotValue );
    }
}

