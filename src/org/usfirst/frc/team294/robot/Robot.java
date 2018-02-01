
package org.usfirst.frc.team294.robot;



import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot; 				//remove the ones that are not used.
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.subsystems.*;
import org.usfirst.frc.team294.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team294.robot.subsystems.DriveTrain;
import org.usfirst.frc.team294.robot.subsystems.Shifter;
import org.usfirst.frc.team294.utilities.FileLog;


public class Robot extends TimedRobot {
	public static final DriveTrain driveTrainSubsystem = new DriveTrain();
	public static final Shifter shifterSubsystem = new Shifter();
	public static OI oi;
	public static boolean allianceSwitchLeft = false;
	public static boolean scaleLeft = false;
	public static boolean opponentSwitchLeft = false;
	public static FileLog log;
	public static Preferences robotPrefs;
	public static int countAtZeroDegrees; 	// Arm potentiometer position at O degrees
	
	Command autonomousCommand;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Create the log file
		log = new FileLog();
		
		// Create the OI
		oi = new OI();
		readPreferences();		// Read preferences next, so that subsystems can use the preference values.
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
		log.writeLogEcho("Autonomous mode started.");
		
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
		autoSelect = oi.readAutoRow();
			
		int startPosition = oi.readStartPosition(); 
		
		
		if (startPosition == 1) {
			programSelected = RobotMap.startingLeftAutoPrograms[autoSelect][fieldLayout];
		} else if (startPosition == 2) {
			programSelected = RobotMap.startingMiddleAutoPrograms[autoSelect][fieldLayout];
		} else {
			programSelected = RobotMap.startingRightAutoPrograms[autoSelect][fieldLayout];
			
		}

		switch (programSelected) {
		case 1 :
			autonomousCommand = new AutoPath1_SameSideScale(startPosition);
			log.writeLogEcho("Ran Auto Path 1 (same side scale), side = " + startPosition);
			break;
		case 2 : 
			autonomousCommand = new AutoPath2_OppositeSideScale(startPosition);
			log.writeLogEcho("Ran Auto Path 2 (opposite side scale), side = " + startPosition);
			break;
		case 3 :
			autonomousCommand = new AutoPath3_SameSideSwitch(startPosition);
			log.writeLogEcho("Ran Auto Path 3 (same side switch), side = " + startPosition);
			break;
		case 4 :
			autonomousCommand = new AutoPath4_OppositeSideSwitchBack(startPosition);
			log.writeLogEcho("Ran Auto Path 4 (opposite side switch back), side = " + startPosition);
			break;
		case 5 :
			autonomousCommand = new AutoPath5_SwitchFromMiddle(allianceSwitchLeft);
			log.writeLogEcho("Ran Auto Path 5 (switch from middle), left = " + allianceSwitchLeft);
		case 6 :
			autonomousCommand = new AutoPath6_OppositeSideSwitchFront(startPosition);
			log.writeLogEcho("Ran Auto Path 6 (opposite side switch front), side = " + startPosition);
			break;
		}
		
		SmartDashboard.putString("Auto path", autonomousCommand.getName());
		SmartDashboard.putNumber("Field selection", fieldLayout);
		SmartDashboard.putNumber("Column Selected", autoSelect);
		
		SmartDashboard.putNumber("Start Position", oi.readStartPosition());
		
		Robot.driveTrainSubsystem.zeroLeftEncoder();
		Robot.driveTrainSubsystem.zeroRightEncoder();
		Robot.driveTrainSubsystem.zeroGyroRotation();

		// schedule the autonomous command
		if (autonomousCommand != null) {
			autonomousCommand.start();
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
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		
		
		log.writeLogEcho("Teleop mode started.");
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
	
	public void readPreferences() {
		//TODO:  Create function to read and set defaults for one number preference, then move most prefs
		//  to calling this function.  This will eliminate much of the duplicate code below.

		//TODO:  For each robot preference:  Use more descriptive names?
		robotPrefs = Preferences.getInstance();

		if (robotPrefs.getDouble("countAtZeroDegrees", 0) == 0) {		//  If field was not set up, set up field
			DriverStation.reportError("Error:  Preferences missing from RoboRio for Arm calibration.", true);
			robotPrefs.putInt("countAtZeroDegrees", 500); //this needs to be changed when we find the new value
		}
		countAtZeroDegrees = robotPrefs.getInt("countAtZeroDegrees", 500);
	}
}
