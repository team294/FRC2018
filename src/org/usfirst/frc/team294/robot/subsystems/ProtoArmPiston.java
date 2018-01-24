package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team294.robot.subsystems.ProtoArmMotor;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
//import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 */
public class ProtoArmPiston extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	
	private final DoubleSolenoid armPiston = new DoubleSolenoid(RobotMap.pneumaticArmPistonIn,RobotMap.pneumaticArmPistonOut);

	
	public void potAngleTest() {      // TODO calculate angle from pot data
		double potAngle = ProtoArmMotor.PotValue;
		SmartDashboard.putNumber("Pot Angle raw count", potAngle);
	}

	
	public void extendPiston() {
		armPiston.set(Value.kForward); // kForward is  extend
	}
	
	public void retractPiston() {
		armPiston.set(Value.kReverse); // kReverse is  retract
	}

	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }
}

