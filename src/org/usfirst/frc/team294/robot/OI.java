package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.ShiftLow;
import org.usfirst.frc.team294.robot.subsystems.ProtoArmMotor;
import org.usfirst.frc.team294.robot.commands.ArmExtend;
import org.usfirst.frc.team294.robot.commands.ArmMotorControl;
import org.usfirst.frc.team294.robot.commands.ArmMotorControlPosition;
import org.usfirst.frc.team294.robot.commands.ArmRetract;
import org.usfirst.frc.team294.robot.commands.ShiftHigh;
import org.usfirst.frc.team294.robot.commands.DriveWithJoystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
	public static Joystick leftJoystick = new Joystick(0); // Left Joystick is in port 0

	public static Joystick rightJoystick = new Joystick(1); // Right Joystick is in port 1
	
	public static Joystick armJoystick = new Joystick(2); // Arm Joystick is in port 2
	
	{
		SmartDashboard.putData("Start Drive Train", new DriveWithJoystick()); // Adds a start Button
		
		SmartDashboard.putData("Retract Arm", new ArmRetract()); // Adds a retract Button
		
		SmartDashboard.putData("Extend Arm", new ArmExtend()); //Adds a extend button
		
		SmartDashboard.putData("Control Arm Motor Joystick", new ArmMotorControl());
		
		SmartDashboard.putData("Control Arm Motor Position", new ArmMotorControlPosition());
//		SmartDashboard.putData("Start Arm", new ArmMotorControl()); // Adds a start Button

		Button leftTrigger = new JoystickButton(leftJoystick, 1);
		Button rightTrigger = new JoystickButton(rightJoystick, 1);
		leftTrigger.whenPressed(new ShiftLow());
		rightTrigger.whenPressed(new ShiftHigh());
	}
}
