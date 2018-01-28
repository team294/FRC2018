package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.ShiftLow;
import org.usfirst.frc.team294.robot.commands.TurnGyro;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units;
import org.usfirst.frc.team294.robot.commands.ShiftHigh;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceProfile;
import org.usfirst.frc.team294.robot.commands.DriveStraightDistanceGyroAngle;
import org.usfirst.frc.team294.robot.commands.DriveWithJoystick;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
	public static Joystick leftJoystick =
			new Joystick(0); // Left Joystick is in port 0

	public static Joystick rightJoystick = new Joystick(1); // Right Joystick is in port 1

	{
		//SmartDashboard.putData("Start Drive Train", new DriveWithJoystick()); // Adds a start Button
		//SmartDashboard.putData("Drive Straight Gyro Angle", new DriveStraightDistanceGyroAngle(150, .75, 0));
/*
		Button leftTrigger = new JoystickButton(leftJoystick, 1);
		Button rightTrigger = new JoystickButton(rightJoystick, 1);
		leftTrigger.whenPressed(new ShiftLow());
		rightTrigger.whenPressed(new ShiftHigh());*/
		//SmartDashboard.putData("Turn heckla small", new TurnGyro(90, Units.Degrees));
		SmartDashboard.putData("DriveStraightDistanceProfile",new DriveStraightDistanceProfile(0,0));
		SmartDashboard.putNumber("DistToTravelDSDG", 150);
	}
}
