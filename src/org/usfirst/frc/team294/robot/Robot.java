
package org.usfirst.frc.team294.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot; 				//remove the ones that are not used.
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.subsystems.*;


public class Robot extends TimedRobot {
	public static final DriveTrain driveTrainSubsystem = new DriveTrain();
	public static final Shifter shifterSubsystem = new Shifter();
	public static OI m_oi;
	public static boolean allianceSwitchLeft = false;
	public static boolean scaleLeft = false;
	public static boolean opponentSwitchLeft = false;
	
//	public static Command[][] autoCommandArray = { {new PnuematicShift(), new RunDriveTrain()} };
	public static Command[][] autoCommandArray = new Command[RobotMap.AUTO_COLS][RobotMap.AUTO_FIELD_LAYOUTS];

	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_oi = new OI();
		/*
		 * auto-config for autonomous
		 */
//		m_chooser.addDefault("Default Auto", new RunDriveTrain());
//		// chooser.addObject("My Auto", new MyAutoCommand());
//		SmartDashboard.putData("Auto mode", m_chooser);
		
		// array chooses which auto program to use based on selected auto path and field layout
//		autoCommandArray[RobotMap.AutoPath.SwitchPriority.ordinal()][RobotMap.AutoFieldLayout.LL.ordinal()] = new AutoPath3_SameSideSwitch();
//		autoCommandArray[RobotMap.AutoPath.SwitchPriority.ordinal()][RobotMap.AutoFieldLayout.LR.ordinal()] = new AutoPath3_SameSideSwitch();
//		autoCommandArray[RobotMap.AutoPath.SwitchPriority.ordinal()][RobotMap.AutoFieldLayout.RL.ordinal()] = new AutoPath1_SameSideScale();
//		autoCommandArray[RobotMap.AutoPath.SwitchPriority.ordinal()][RobotMap.AutoFieldLayout.RR.ordinal()] = new AutoPath2_OppositeSideScale();
//
//		autoCommandArray[RobotMap.AutoPath.BothSwitchesMiddle.ordinal()][RobotMap.AutoFieldLayout.LL.ordinal()] = new AutoPath1_SameSideScale();
//		autoCommandArray[RobotMap.AutoPath.BothSwitchesMiddle.ordinal()][RobotMap.AutoFieldLayout.LR.ordinal()] = new AutoPath2_OppositeSideScale();
//		autoCommandArray[RobotMap.AutoPath.BothSwitchesMiddle.ordinal()][RobotMap.AutoFieldLayout.RL.ordinal()] = new AutoPath1_SameSideScale();
//		autoCommandArray[RobotMap.AutoPath.BothSwitchesMiddle.ordinal()][RobotMap.AutoFieldLayout.RR.ordinal()] = new AutoPath2_OppositeSideScale();
//
//		autoCommandArray[RobotMap.AutoPath.ScaleOnly.ordinal()][RobotMap.AutoFieldLayout.LL.ordinal()] = new AutoPath5_SwitchFromMiddle();
//		autoCommandArray[RobotMap.AutoPath.ScaleOnly.ordinal()][RobotMap.AutoFieldLayout.LR.ordinal()] = new AutoPath5_SwitchFromMiddle();
//		autoCommandArray[RobotMap.AutoPath.ScaleOnly.ordinal()][RobotMap.AutoFieldLayout.RL.ordinal()] = new AutoPath5_SwitchFromMiddle();
//		autoCommandArray[RobotMap.AutoPath.ScaleOnly.ordinal()][RobotMap.AutoFieldLayout.RR.ordinal()] = new AutoPath5_SwitchFromMiddle();
//		
//		autoCommandArray[RobotMap.AutoPath.ScalePriority.ordinal()][RobotMap.AutoFieldLayout.LL.ordinal()] = new AutoPath3_SameSideSwitch();
//		autoCommandArray[RobotMap.AutoPath.ScalePriority.ordinal()][RobotMap.AutoFieldLayout.LR.ordinal()] = new AutoPath3_SameSideSwitch();
//		autoCommandArray[RobotMap.AutoPath.ScalePriority.ordinal()][RobotMap.AutoFieldLayout.RL.ordinal()] = new AutoPath1_SameSideScale();
//		autoCommandArray[RobotMap.AutoPath.ScalePriority.ordinal()][RobotMap.AutoFieldLayout.RR.ordinal()] = new AutoPath4_OppositeSideSwitch();

	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
			
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		
		
		if(gameData.charAt(0) == 'L')
		{
			SmartDashboard.putBoolean("Close Switch Left", true);
			SmartDashboard.putBoolean("Close Switch Right", false);
			allianceSwitchLeft = true;
			//Put left auto code here
		} else {
			SmartDashboard.putBoolean("Close Switch Right", true);
			SmartDashboard.putBoolean("Close Switch Left", false);
			allianceSwitchLeft = false;
			//Put right auto code here
		}
		
		if(gameData.charAt(1) == 'L')
		{
			SmartDashboard.putBoolean("Scale Left", true);
			SmartDashboard.putBoolean("Scale Right", false);
			scaleLeft = true;
			//Put left auto code here
		} else {
			SmartDashboard.putBoolean("Scale Right", true);
			SmartDashboard.putBoolean("Scale Left", false);
			scaleLeft = false;
			//Put right auto code here
		}
		
		if(gameData.charAt(2) == 'L')
		{
			SmartDashboard.putBoolean("Far Switch Left", true);
			SmartDashboard.putBoolean("Far Switch Right", false);
			opponentSwitchLeft = true;
			//Put left auto code here
		} else {
			SmartDashboard.putBoolean("Far Switch Right", true);
			SmartDashboard.putBoolean("Far Switch Left", false);
			opponentSwitchLeft = false;
			//Put right auto code here
		}
		
		DriverStation.Alliance color;
		color = DriverStation.getInstance().getAlliance();
		
		if(color == DriverStation.Alliance.Blue)
		{
			SmartDashboard.putBoolean("Alliance Color", true);
		} else {
			SmartDashboard.putBoolean("Alliance Color", false);
		}
		
		
		int fieldLayout, autoSelect;

		if (gameData.startsWith("LL")) 
			fieldLayout = RobotMap.AutoFieldLayout.LL.ordinal();
		else if (gameData.startsWith("LR"))
			fieldLayout = RobotMap.AutoFieldLayout.LR.ordinal();
		else if (gameData.startsWith("RL"))
			fieldLayout = RobotMap.AutoFieldLayout.RL.ordinal();
		else
			fieldLayout = RobotMap.AutoFieldLayout.RR.ordinal();

		int programSelected;
		autoSelect = m_oi.readAutoRow();
			
		int startPosition = m_oi.readStartPosition(); 
		
		
		if (startPosition == 1) {
			programSelected = RobotMap.startingLeftAutoPrograms[autoSelect][fieldLayout];
		} else if (startPosition == 2) {
			programSelected = RobotMap.startingMiddleAutoPrograms[autoSelect][fieldLayout];
		} else {
			programSelected = RobotMap.startingRightAutoPrograms[autoSelect][fieldLayout];
			
		}

		switch (programSelected) {
		case 1 :
			m_autonomousCommand = new AutoPath1_SameSideScale(startPosition);
			break;
		case 2 : 
			m_autonomousCommand = new AutoPath2_OppositeSideScale(startPosition);
			break;
		case 3 :
			m_autonomousCommand = new AutoPath3_SameSideSwitch(startPosition);
			break;
		case 4 :
			m_autonomousCommand = new AutoPath4_OppositeSideSwitch(startPosition);
			break;
		case 5 :
			m_autonomousCommand = new AutoPath5_SwitchFromMiddle(allianceSwitchLeft);

			
		}
		
	// NOTE: NEED TO FIX COMMANDS AND COMMAND SEQUENCES (deleted AutoPath1_SameSideScale command but left sequence)
		
//		m_autonomousCommand = autoCommandArray[autoSelect][fieldLayout];
		SmartDashboard.putString("Auto path", m_autonomousCommand.getName());
		SmartDashboard.putNumber("Field selection", fieldLayout);
		SmartDashboard.putNumber("Column Selected", autoSelect);
		
		SmartDashboard.putNumber("Start Position", m_oi.readStartPosition());
		

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}
