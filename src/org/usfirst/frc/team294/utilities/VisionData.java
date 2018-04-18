package org.usfirst.frc.team294.utilities;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionData {
	public double RPiX ;//X coordinate from Raspberry Pi
	public double RPiY;//Y coordinate from Raspberry Pi
	
	private NetworkTableEntry xCoord;
	private NetworkTableEntry yCoord;

	/**
	 * Creates a VisionData object and connects to the RPi keys in the NetworkTable.
	 */
	public VisionData() {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		inst.startClientTeam(294);
		NetworkTable pi = inst.getTable("Pi");

		xCoord = pi.getEntry("X");
		yCoord = pi.getEntry("Y");		
	}
	
	/**
	 * Reads NetworkTable data from the camera and saves the
	 * results in RPiX and RPiY.
	 */
	public void readCameraData() {
		RPiX = xCoord.getDouble(-1);		
		RPiY = yCoord.getDouble(-1);
	}

	/**
	 * Gets the angle the cube is at, relative to the current robot heading.
	 * @return angle in degrees (+ to right, - to left, 0 if no cube found)
	 */
	public double getCubeAngleRelative() {
		double distanceFromCenter;// tells the distance from the center of the screen
		
		Robot.visionData.readCameraData();
		if (RPiX == -1 || (RPiX >= 310 && RPiX <= 330)) {
			return 0.0;
		} else {
			distanceFromCenter = RPiX - 320;
			return Math.atan(distanceFromCenter / (752) ) * (180 / Math.PI);   // Nom dist was 470
		}
	}
	
}
