package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.ShiftLow;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units;
import org.usfirst.frc.team294.robot.commands.ShiftHigh;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceGyro;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistance;
import org.usfirst.frc.team294.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
	public static Joystick leftJoystick = new Joystick(0); // Left Joystick is in port 0

	public static Joystick rightJoystick = new Joystick(1); // Right Joystick is in port 1

	{
		SmartDashboard.putData("Start Drive Train", new DriveWithJoystick()); // Adds a start Button
		SmartDashboard.putData("Drive Straight Gyro", new DriveStraightDistanceGyro(150, .4));
		SmartDashboard.putData("Drive Straight", new DriveStraightDistance(121, .9)); 
		
		Button leftTrigger = new JoystickButton(leftJoystick, 1);
		Button rightTrigger = new JoystickButton(rightJoystick, 1);
		leftTrigger.whenPressed(new ShiftLow());
		rightTrigger.whenPressed(new ShiftHigh());
		SmartDashboard.putData("Turn heckla small", new TurnGyro(90,Units.Degrees));
		SmartDashboard.putData("Turn heckla small NEGATIVE2", new TurnGyro());
		SmartDashboard.putNumber("Gyro Turn Angle Input", 90);
	}
}
