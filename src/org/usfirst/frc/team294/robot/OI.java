package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

	


public class OI {
	public static Joystick leftJoystick = new Joystick(0); // Left Joystick is in port 0

	public static Joystick rightJoystick = new Joystick(1); // Right Joystick is in port 1

	public static Joystick armJoystick = new Joystick(2); // Arm Joystick is in port 2
	
	SendableChooser<Integer> chooser_autoPlan = new SendableChooser<>();
	SendableChooser<Integer> chooser_startPosition = new SendableChooser<>();

	public OI() {
		// Initialize our auto plan chooser
		chooser_autoPlan.addDefault("Closest, far/far = scale", 0);
		chooser_autoPlan.addObject("Closest, far/far = switch front", 1);
		chooser_autoPlan.addObject("Closest, far/far = switch back", 2);
		chooser_autoPlan.addObject("Scale only", 3);
		chooser_autoPlan.addObject("Switch only (middle)", 4);
		SmartDashboard.putData("Auto Plan Selection", chooser_autoPlan);
		
		// Initialize our position chooser
		chooser_startPosition.addDefault("- choose from below -", 0);
		chooser_startPosition.addObject("Left", 1);
		chooser_startPosition.addObject("Middle", 2);
		chooser_startPosition.addObject("Right", 3);
		SmartDashboard.putData("Start Position Selection", chooser_startPosition);
	
		SmartDashboard.putData("Start Drive Train", new DriveWithJoystick()); // Adds a start Button
		
		SmartDashboard.putData("Retract Arm", new ArmRetract()); // Adds a retract Button
		
		SmartDashboard.putData("Extend Arm", new ArmExtend()); //Adds a extend button
		
		SmartDashboard.putData("Control Arm Motor Joystick", new ArmMotorControl());
				
		SmartDashboard.putData("Set Arm Position", new SetArmFromSmartDashboard());
		SmartDashboard.putNumber("set Arm Angle", 0);

		Button leftTrigger = new JoystickButton(leftJoystick, 1);
		Button rightTrigger = new JoystickButton(rightJoystick, 1);
		leftTrigger.whenPressed(new ShiftLow());
		rightTrigger.whenPressed(new ShiftHigh());
		
		SmartDashboard.putData("Photo Switch", new ReadPhotoSwitch()); // For use with the intake
		SmartDashboard.putData("Pick Up Cube", new CubePickUp());
		SmartDashboard.putData("Release Cube", new CubeLetGo());
		SmartDashboard.putData("Shoot Out Cube", new CubeShootOut());

	}
	
	public int readAutoPlan() {
		return chooser_autoPlan.getSelected();
	}
	
	public int readStartPosition() {
		return chooser_startPosition.getSelected();
	}
}
