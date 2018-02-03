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
	public void openClaw() {
		clawPiston.set(Value.kForward); // kForward is extend
	}

	public void closeClaw() {
		clawPiston.set(Value.kReverse); // kReverse is retract
	}

	public void setClawMotorToPercentPower(double leftPercent, double rightPercent) {
		clawMotorLeft.set(ControlMode.PercentOutput, leftPercent);
		clawMotorRight.set(ControlMode.PercentOutput, rightPercent);
		System.out.println("Left Claw motor " + clawMotorLeft.getDeviceID() + " set to percent " + leftPercent + ", output "
				+ clawMotorLeft.getMotorOutputVoltage() + " V," + clawMotorLeft.getOutputCurrent() + " A, Bus at "
				+ clawMotorLeft.getBusVoltage() + " V");
		System.out.println("Right Claw motor " + clawMotorRight.getDeviceID() + " set to percent " + rightPercent
				+ ", output " + clawMotorRight.getMotorOutputVoltage() + " V," + clawMotorRight.getOutputCurrent()
				+ " A, Bus at " + clawMotorRight.getBusVoltage() + " V");
		SmartDashboard.putNumber("Left Claw Motor Percent:", leftPercent);
		SmartDashboard.putNumber("Right Claw Motor Percent:", rightPercent);
	}

	/* public double getClawMotorPower() {
		double voltage = (clawMotorLeft.getMotorOutputPercent() + clawMotorRight.getMotorOutputPercent()) / 2;
		SmartDashboard.putNumber("Arm Motor Output Voltage", voltage);
		return voltage;
	} */

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}
