package org.usfirst.frc.team294.utilities;

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
		
}
