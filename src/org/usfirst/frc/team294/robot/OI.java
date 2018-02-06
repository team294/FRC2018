package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.commands.autoroutines.*;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units;

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

	{
		SmartDashboard.putData("AutoTest1",new AutoTest1());
	
		// Initialize our auto plan chooser
		chooser_autoPlan.addDefault("do Closest, if both far do scale", 0);
		chooser_autoPlan.addObject("do Closest, if both far do switch from front", 1);
		chooser_autoPlan.addObject("do closest, if both far do switch from back", 2);
		chooser_autoPlan.addObject("do Scale only", 3);
		chooser_autoPlan.addObject("do Switch only from middle", 4);
		chooser_autoPlan.addObject("Go to baseline", 5);
		SmartDashboard.putData("Auto Plan Selection", chooser_autoPlan);
		
		// Initialize our position chooser
		chooser_startPosition.addDefault("- choose from below -", 0);
		chooser_startPosition.addObject("Left", 1);
		chooser_startPosition.addObject("Middle", 2);
		chooser_startPosition.addObject("Right", 3);
		SmartDashboard.putData("Start Position Selection", chooser_startPosition);
	
		SmartDashboard.putData("Start Drive Train", new DriveWithJoystick()); // Adds a start Button
		SmartDashboard.putData("Drive Straight Gyro Angle", new DriveStraightDistanceGyroAngle(150, .75, 0));
		SmartDashboard.putData("Retract Arm", new ArmRetract()); // Adds a retract Button
		SmartDashboard.putData("Extend Arm", new ArmExtend()); // Adds a extend button
		SmartDashboard.putData("Control Arm Motor Joystick", new ArmMotorControl());
		SmartDashboard.putData("Set Arm Position", new SetArmFromSmartDashboard());
		SmartDashboard.putNumber("set Arm Angle", 0);
		Button leftTrigger = new JoystickButton(leftJoystick, 1);
		Button rightTrigger = new JoystickButton(rightJoystick, 1);
		leftTrigger.whenPressed(new ShiftLow());
		rightTrigger.whenPressed(new ShiftHigh());
		SmartDashboard.putData("Turn heckla small", new TurnGyro(90, Units.Degrees));
		SmartDashboard.putData("DriveStraightDistanceProfile", new DriveStraightDistanceProfile(0, 0));
		SmartDashboard.putNumber("DistToTravelDSDG", 150);
		SmartDashboard.putData(" ProfileTest", new DriveStraightDistanceEllipse(100, 1000, 0));

	}
	
	public int readAutoPlan() {
		return chooser_autoPlan.getSelected();
	}
	
	public int readStartPosition() {
		return chooser_startPosition.getSelected();
	}
}
