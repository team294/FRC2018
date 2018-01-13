package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.PnuematicShift;
import org.usfirst.frc.team294.robot.commands.RunDriveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
	public static Joystick leftJoystick = new Joystick(0); //Left Joystick is in port 0

	public static Joystick rightJoystick = new Joystick(1); //Right Joystick is in port 1

	{
		SmartDashboard.putData("Start Drive Train", new RunDriveTrain()); //Adds a start Button
		SmartDashboard.putData("Start Shifter", new PnuematicShift()); //Adds a start Button
	}
}
