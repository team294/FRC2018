package org.usfirst.frc.team294.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot; //remove the ones that are not used.
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.subsystems.*;
import org.usfirst.frc.team294.robot.commands.DriveWithJoystick;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath1_SameSideScale;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath2_OppositeSideScale;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath3_SameSideSwitch;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath4_OppositeSideSwitchBack;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath5_SwitchFromMiddle;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath6_OppositeSideSwitchFront;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath7_Baseline;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoTest1;
import org.usfirst.frc.team294.robot.subsystems.DriveTrain;
import org.usfirst.frc.team294.robot.subsystems.ProtoArmMotor;
import org.usfirst.frc.team294.robot.subsystems.ProtoArmPiston;
import org.usfirst.frc.team294.robot.subsystems.Shifter;
import org.usfirst.frc.team294.utilities.FileLog;

public class Robot extends TimedRobot {
	// Subsystem objects
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
	public static String gameData;
	public static int armCalZero; // Arm potentiometer position at O degrees
	public static int armCal90Deg; // Arm potentiometer position at 90 degrees

	private Command autonomousCommand;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		driveTrain = new DriveTrain();
		shifter = new Shifter();
		protoArmPiston = new ProtoArmPiston();
		protoArmMotor = new ProtoArmMotor();
		// Create the log file
		log = new FileLog();
		// Create the OI
		oi = new OI();
		readPreferences(); // Read preferences next, so that subsystems can use the preference values.
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
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
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString code to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons to
	 * the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.charAt(0) == 'L') {
			SmartDashboard.putBoolean("Close Switch Left", true);
			SmartDashboard.putBoolean("Close Switch Right", false);
			allianceSwitchLeft = true;
			// Put left auto code here
		} else {
			SmartDashboard.putBoolean("Close Switch Right", true);
			SmartDashboard.putBoolean("Close Switch Left", false);
			allianceSwitchLeft = false;
			// Put right auto code here
		}

		if (gameData.charAt(1) == 'L') {
			SmartDashboard.putBoolean("Scale Left", true);
			SmartDashboard.putBoolean("Scale Right", false);
			scaleLeft = true;
			// Put left auto code here
		} else {
			SmartDashboard.putBoolean("Scale Right", true);
			SmartDashboard.putBoolean("Scale Left", false);
			scaleLeft = false;
			// Put right auto code here
		}

		if (gameData.charAt(2) == 'L') {
			SmartDashboard.putBoolean("Far Switch Left", true);
			SmartDashboard.putBoolean("Far Switch Right", false);
			opponentSwitchLeft = true;
			// Put left auto code here
		} else {
			SmartDashboard.putBoolean("Far Switch Right", true);
			SmartDashboard.putBoolean("Far Switch Left", false);
			opponentSwitchLeft = false;
			// Put right auto code here
		}

		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();

		if (color == DriverStation.Alliance.Blue) {
			SmartDashboard.putBoolean("Alliance Color", true);
		} else {
			SmartDashboard.putBoolean("Alliance Color", false);
		}
		driveTrain.zeroLeftEncoder();
		driveTrain.zeroRightEncoder();
		driveTrain.zeroGyroRotation();
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		 * switch(autoSelected) { case "My Auto": autonomousCommand = new
		 * MyAutoCommand(); break; case "Default Auto": default: autonomousCommand = new
		 * ExampleCommand(); break; }
		 */
		Command m_autonomousCommand = null;

		int fieldLayout, autoPlan;
		if (gameData.startsWith("LL"))
			fieldLayout = RobotMap.AutoFieldLayout.LL.ordinal();
		else if (gameData.startsWith("LR"))
			fieldLayout = RobotMap.AutoFieldLayout.LR.ordinal();
		else if (gameData.startsWith("RL"))
			fieldLayout = RobotMap.AutoFieldLayout.RL.ordinal();
		else
			fieldLayout = RobotMap.AutoFieldLayout.RR.ordinal();

		int programSelected;
		autoPlan = oi.readAutoPlan();

		int startPosition = oi.readStartPosition();

		if (startPosition == 1) {
			programSelected = RobotMap.startingLeftAutoPrograms[autoPlan][fieldLayout];
		} else if (startPosition == 2) {
			programSelected = RobotMap.startingMiddleAutoPrograms[autoPlan][fieldLayout];
		} else {
			programSelected = RobotMap.startingRightAutoPrograms[autoPlan][fieldLayout];

		}
		switch (programSelected) {
		case 1:
			autonomousCommand = new AutoPath1_SameSideScale(startPosition);
			log.writeLogEcho("Ran Auto Path 1 (same side scale), side = " + startPosition);
			break;
		case 2:
			autonomousCommand = new AutoPath2_OppositeSideScale(startPosition);
			log.writeLogEcho("Ran Auto Path 2 (opposite side scale), side = " + startPosition);
			break;
		case 3:
			autonomousCommand = new AutoPath3_SameSideSwitch(startPosition);
			log.writeLogEcho("Ran Auto Path 3 (same side switch), side = " + startPosition);
			break;
		case 4:
			autonomousCommand = new AutoPath4_OppositeSideSwitchBack(startPosition);
			log.writeLogEcho("Ran Auto Path 4 (opposite side switch back), side = " + startPosition);
			break;
		case 5:
			autonomousCommand = new AutoPath5_SwitchFromMiddle(allianceSwitchLeft);
			log.writeLogEcho("Ran Auto Path 5 (switch from middle), left = " + allianceSwitchLeft);
			break;
		case 6:
			autonomousCommand = new AutoPath6_OppositeSideSwitchFront(startPosition);
			log.writeLogEcho("Ran Auto Path 6 (opposite side switch front), side = " + startPosition);
			break;
		case 7:
			autonomousCommand = new AutoPath7_Baseline(startPosition);
			log.writeLogEcho("Ran Auto Path 7 (Go to baseline), side = " + startPosition);
			break;
		}

		SmartDashboard.putString("Auto path", autonomousCommand.getName());
		SmartDashboard.putNumber("Auto program #", programSelected);
		SmartDashboard.putNumber("Auto field selection", fieldLayout);
		SmartDashboard.putNumber("Auto plan selected", autoPlan);
		SmartDashboard.putNumber("Auto start position", startPosition);

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

		this.driveTrain.zeroGyroRotation(); // todo remove later
		this.driveTrain.setFieldPositionX(0); // todo remove later
		this.driveTrain.setFieldPositionY(0); // todo remove later

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
		// TODO: Create function to read and set defaults for one number preference,
		// then move most prefs
		// to calling this function. This will eliminate much of the duplicate code
		// below.

		// TODO: For each robot preference: Use more descriptive names?
		robotPrefs = Preferences.getInstance();

		if (robotPrefs.getDouble("calibrationZeroDegrees", 0) == 0) { // If field was not set up, set up field
			DriverStation.reportError("Error:  Preferences missing from RoboRio for Arm calibration.", true);
			robotPrefs.putInt("calibrationZeroDegrees", -245); // Value may need to be changed based on specifics of
																// robot
		}
		armCalZero = robotPrefs.getInt("calibrationZeroDegrees", -245);
		armCal90Deg = robotPrefs.getInt("calibration90Degrees", -195);
	}
}
