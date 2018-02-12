package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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

	private final TalonSRX clawMotorLeft = new TalonSRX(RobotMap.clawMotorLeft);
	private final TalonSRX clawMotorRight = new TalonSRX(RobotMap.clawMotorRight);
	
	public Claw() {
		// Configure talons
		clawMotorLeft.set(ControlMode.PercentOutput, 0);
		clawMotorLeft.setNeutralMode(NeutralMode.Coast);
		clawMotorLeft.enableVoltageCompensation(true);
		clawMotorLeft.configVoltageCompSaturation(11.0, 0);
		clawMotorLeft.configOpenloopRamp(0.2, 0);

		clawMotorRight.set(ControlMode.PercentOutput, 0);
		clawMotorRight.setNeutralMode(NeutralMode.Coast);
		clawMotorRight.enableVoltageCompensation(true);
		clawMotorRight.configVoltageCompSaturation(11.0, 0);
		clawMotorRight.configOpenloopRamp(0.2, 0);
	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	public void openClaw() {
		double currentAngle = Robot.protoArmMotor.getArmDegrees();
		if (currentAngle <= RobotMap.angleClawCloseHigh && currentAngle >= RobotMap.angleClawCloseLow) {
		}
		else {
			clawPiston.set(Value.kForward); // kForward is extend
		}
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
