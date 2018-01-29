package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.ShiftLow;
import org.usfirst.frc.team294.robot.commands.ShiftHigh;
import org.usfirst.frc.team294.robot.commands.DriveWithJoystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

	


public class OI {
	public static Joystick leftJoystick = new Joystick(0); // Left Joystick is in port 0

	public static Joystick rightJoystick = new Joystick(1); // Right Joystick is in port 1
	SendableChooser<Integer> chooser_autoColumn = new SendableChooser<>();
	SendableChooser<Integer> chooser_startPosition = new SendableChooser<>();


	
	public OI() {
		// Initialize our auto column chooser
		chooser_autoColumn.addDefault("Switch priority", 0);
		chooser_autoColumn.addObject("Both switches middle", 1);
		chooser_autoColumn.addObject("Scale only", 2);
		chooser_autoColumn.addObject("Scale priority", 3);
		SmartDashboard.putData("Auto Column Selection", chooser_autoColumn);
		
		// Initialize our position chooser
		chooser_startPosition.addDefault("- choose from below -", 0);
		chooser_startPosition.addObject("Left", 1);
		chooser_startPosition.addObject("Middle", 2);
		chooser_startPosition.addObject("Right", 3);
		SmartDashboard.putData("Start Position Selection", chooser_startPosition);
	
		SmartDashboard.putData("Start Drive Train", new DriveWithJoystick()); // Adds a start Button
		Button leftTrigger = new JoystickButton(leftJoystick, 1);
		Button rightTrigger = new JoystickButton(rightJoystick, 1);
		leftTrigger.whenPressed(new ShiftLow());
		rightTrigger.whenPressed(new ShiftHigh());

	}
	
	public int readAutoRow() {
		return chooser_autoColumn.getSelected();
	}
	
	public int readStartPosition() {
		return chooser_startPosition.getSelected();
	}
}
