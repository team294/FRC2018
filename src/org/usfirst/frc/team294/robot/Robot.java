
package org.usfirst.frc.team294.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot; 				//remove the ones that are not used.
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team294.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team294.robot.subsystems.DriveTrain;
import org.usfirst.frc.team294.robot.subsystems.ProtoArmMotor;
import org.usfirst.frc.team294.robot.subsystems.ProtoArmPiston;
import org.usfirst.frc.team294.robot.subsystems.Shifter;
import org.usfirst.frc.team294.utilities.FileLog;


public class Robot extends TimedRobot {
	
	public static DriveTrain driveTrain;
	public static Shifter shifter;
	public static ProtoArmPiston protoArmPiston;
	public static ProtoArmMotor protoArmMotor;
	public static OI oi;
	
	public static boolean allianceSwitchLeft = false;
	public static boolean scaleLeft = false;
	public static boolean opponentSwitchLeft = false;
	public static FileLog log;
	public static Preferences robotPrefs;
	public static int countAtZeroDegrees; 	// Arm potentiometer position at O degrees
	

	//Command m_autonomousCommand;
	//SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {

		driveTrain = new DriveTrain();
		shifter = new Shifter();
		protoArmPiston = new ProtoArmPiston();
		protoArmMotor = new ProtoArmMotor();
		oi = new OI();

		// Create the log file
		log = new FileLog();
		
		// Create the OI
		readPreferences();		// Read preferences next, so that subsystems can use the preference values.

		/*
		 * auto-config for autonomous
		 */
		//m_chooser.addDefault("Default Auto", new DriveWithJoystick());
		// chooser.addObject("My Auto", new MyAutoCommand());
		//SmartDashboard.putData("Auto mode", m_chooser);
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
		
		String gameData = DriverStation.getInstance().getGameSpecificMessage();
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
		
		String gameData = DriverStation.getInstance().getGameSpecificMessage();

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
		
		
		//m_autonomousCommand = m_chooser.getSelected();
    
		driveTrain.zeroLeftEncoder();
		driveTrain.zeroRightEncoder();
		driveTrain.zeroGyroRotation();
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		// schedule the autonomous command (example)
		/*if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}*/
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
		/*if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}*/
		
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
