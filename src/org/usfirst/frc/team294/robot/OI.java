package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.PnuematicShift;
import org.usfirst.frc.team294.robot.commands.RunDriveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
	SendableChooser<Integer> chooser_autoColumn = new SendableChooser<>();

	public static Joystick leftJoystick = new Joystick(0); //Left Joystick is in port 0

	public static Joystick rightJoystick = new Joystick(1); //Right Joystick is in port 1

	public OI() {
		// Initialize our auto column chooser
		chooser_autoColumn.addDefault("Switch priority", 1);
		chooser_autoColumn.addObject("Scale priority", 2);
		SmartDashboard.putData("Auto Column Selection", chooser_autoColumn);
	
		SmartDashboard.putData("Start Drive Train", new RunDriveTrain()); //Adds a start Button
		SmartDashboard.putData("Start Shifter", new PnuematicShift()); //Adds a start Button
	}
	
	public int readAutoColumn() {
		return chooser_autoColumn.getSelected();
	}
}
