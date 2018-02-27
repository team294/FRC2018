package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PressureSensor extends Subsystem {
	private AnalogInput pressure = new AnalogInput(RobotMap.pressureSensor);
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public double getVoltage() {
    	double voltage;
    	voltage = pressure.getVoltage();
    	return voltage;
    }
    
    public  int getPressure() {			// integer is all the resolution we need for display
    	double voltage = getVoltage();
    	double inputVoltage = 4.8;
    	double pressure = (250 * (voltage / inputVoltage)) - 25;
    	return (int) pressure;
    }

    public void periodic() {
    	SmartDashboard.putNumber("High Pressure", getPressure()); 
    }
}