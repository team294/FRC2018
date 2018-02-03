package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Claw extends Subsystem {

	private final DoubleSolenoid clawPiston = new DoubleSolenoid(RobotMap.pneumaticClawPistonIn,
			RobotMap.pneumaticClawPistonOut);

	private final WPI_TalonSRX clawMotorLeft = new WPI_TalonSRX(RobotMap.clawMotorLeft);
	private final WPI_TalonSRX clawMotorRight = new WPI_TalonSRX(RobotMap.clawMotorRight);

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void extendPiston() {
		clawPiston.set(Value.kForward); // kForward is extend
	}

	public void retractPiston() {
		clawPiston.set(Value.kReverse); // kReverse is retract
	}

	public void setClawMotorToPercentPower(double percent) {
		clawMotorLeft.set(ControlMode.PercentOutput, (percent * .8));
		clawMotorRight.set(ControlMode.PercentOutput, percent);
		System.out.println("Left Claw motor " + clawMotorLeft.getDeviceID() + " set to percent " + percent + ", output "
				+ clawMotorLeft.getMotorOutputVoltage() + " V," + clawMotorLeft.getOutputCurrent() + " A, Bus at "
				+ clawMotorLeft.getBusVoltage() + " V");
		System.out.println("Right Claw motor " + clawMotorRight.getDeviceID() + " set to percent " + percent
				+ ", output " + clawMotorRight.getMotorOutputVoltage() + " V," + clawMotorRight.getOutputCurrent()
				+ " A, Bus at " + clawMotorRight.getBusVoltage() + " V");
		SmartDashboard.putNumber("Claw Motors Percent", percent);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
