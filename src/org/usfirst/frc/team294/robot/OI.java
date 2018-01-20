package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.PnuematicShift;
import org.usfirst.frc.team294.robot.commands.RunDriveTrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {
	SendableChooser<Integer> chooser_autoColumn = new SendableChooser<>();
	SendableChooser<Integer> chooser_startPosition = new SendableChooser<>();


	public static Joystick leftJoystick = new Joystick(0); //Left Joystick is in port 0

	public static Joystick rightJoystick = new Joystick(1); //Right Joystick is in port 1

	public OI() {
		// Initialize our auto column chooser
		chooser_autoColumn.addDefault("Switch priority", 1);
		chooser_autoColumn.addObject("Both switches middle", 2);
		chooser_autoColumn.addObject("Scale only", 3);
		chooser_autoColumn.addObject("Scale priority", 4);
		SmartDashboard.putData("Auto Column Selection", chooser_autoColumn);
		
		// Initialize our position chooser
		chooser_startPosition.addDefault("Left", 1);
		chooser_startPosition.addObject("Middle", 2);
		chooser_startPosition.addObject("Right", 3);
		SmartDashboard.putData("Start Position Selection", chooser_startPosition);

	
		SmartDashboard.putData("Start Drive Train", new RunDriveTrain()); //Adds a start Button
		SmartDashboard.putData("Start Shifter", new PnuematicShift()); //Adds a start Button
	}
	
	public int readAutoColumn() {
		return chooser_autoColumn.getSelected();
	}
	
	public int readStartPosition() {
		return chooser_startPosition.getSelected();
	}
}
