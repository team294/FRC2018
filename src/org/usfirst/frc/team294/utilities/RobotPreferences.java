package org.usfirst.frc.team294.utilities;

import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Preferences;

public class RobotPreferences {

	private final Preferences prefs;
	/*
	 * all of the robot preferences
	 */
	public double armCalZero;// = robotPrefs.getInt("calibrationZeroDegrees", -245);
	public boolean prototypeRobot;// = robotPrefs.getBoolean("prototypeRobot", false); // true if testing code on a prototype
	public int armCal90Deg;// = robotPrefs.getInt("calibration90Degrees", -195);
	public boolean driveDirection;// = robotPrefs.getBoolean("driveDirection", true);
	public double wheelCircumference;// = robotPrefs.getDouble("wheelDiameter", 6.18) * Math.PI;
	
	public RobotPreferences() {
		prefs = Preferences.getInstance();
		refresh();
	}
	public void refresh() {
		armCalZero = prefs.getDouble("calibrationZeroDegrees", -245);
		prototypeRobot = prefs.getBoolean("prototypeRobot", false); // true if testing code on a prototype
		armCal90Deg = prefs.getInt("calibration90Degrees", -195);
		driveDirection = prefs.getBoolean("driveDirection", true);
		wheelCircumference = prefs.getDouble("wheelDiameter", 6.18) * Math.PI;
	}
	public String getString(String k) {
		return getString(k, null);
	}
	public String getString(String k, String d) {
		return prefs.getString(k, d);
	}
	public int getInt(String k) {
		return getInt(k, 0);
	}
	public int getInt(String k, int d) {
		return prefs.getInt(k, d);
	}
	public double getDouble(String k, double d) {
		return prefs.getDouble(k, d);
	}
	public double getDouble(String k) {
		return getDouble(k, 0);
	}
}
