
package org.usfirst.frc.team294.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot; //remove the ones that are not used.
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.subsystems.*;
import org.usfirst.frc.team294.utilities.AutoSelection;
import org.usfirst.frc.team294.utilities.FileLog;
import org.usfirst.frc.team294.utilities.RobotPreferences;

public class Robot extends TimedRobot {

	// Subsystem objects
	public static DriveTrain driveTrain;
	public static Shifter shifter;
	public static ArmPiston armPiston;
	public static ArmMotor armMotor;
	public static Claw claw;
	public static Intake intake;
	public static OI oi;
	public static Climb climb;

	public static FileLog log;
	public static RobotPreferences robotPrefs;
	public static AutoSelection autoSelection;

	public static boolean prototypeRobot; // Set true if using code for prototype, false for practice and competition
	public static boolean driveDirection; // true for reversed

	public NetworkTableInstance networkTables;
	public NetworkTable coDisplay;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Create the log file first, so that any other code can use the file log
		log = new FileLog();
		
		// Read robot preferences **before** creating subsystems, so subsytems can use the preferences
		robotPrefs = new RobotPreferences();
				
		// Create subsystems
		driveTrain = new DriveTrain();
		shifter = new Shifter();
		armPiston = new ArmPiston();
		armMotor = new ArmMotor();
		claw = new Claw();
		climb = new Climb();
		intake = new Intake();

		// Create auto selection utility
		autoSelection = new AutoSelection();

		// Network Tables for driver's display
		networkTables = NetworkTableInstance.getDefault();
		coDisplay = networkTables.getTable("coDisplay"); // I think this will work, just need to send value to it

		// Create the OI last, so that it can use commands that call subsystems
		oi = new OI();
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
		log.writeLogEcho("Autonomous mode started.");

		autoSelection.readGameData();

		driveTrain.zeroLeftEncoder();
		driveTrain.zeroRightEncoder();
		driveTrain.zeroGyroRotation();

		// schedule the autonomous command
		if (autoSelection.autonomousCommand != null) {
			Command shiftLow = new Shift(false);
			shiftLow.start();
			autoSelection.autonomousCommand.start();
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		driveTrain.getGyroRotation();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autoSelection.autonomousCommand != null) {
			autoSelection.autonomousCommand.cancel();
		}
		driveTrain.zeroGyroRotation(); // todo remove later
		driveTrain.setFieldPositionX(0); // todo remove later
		driveTrain.setFieldPositionY(0); // todo remove later

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
}
