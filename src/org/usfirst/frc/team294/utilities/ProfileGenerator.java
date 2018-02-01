package org.usfirst.frc.team294.utilities;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ProfileGenerator {
	
	public double currentPosition;
	public double finalPosition;

	public double currentVelocity;
	public double maxVelocity;

	public double currentAcceleration;
	public double maxAcceleration;

	public double dt;
	public double totalTime;
		
	private double directionSign;
	
	private long startTime, currentTime;

	/**
	 * Creates a new profile generator, starting now
	 * @param initialPosition in inches
	 * @param finalPosition in inches
	 * @param initialVelocity in inches per second
	 * @param maxVelocity in inches per second
	 * @param maxAcceleration in inches per second^2
	 */
	public ProfileGenerator(double initialPosition, double finalPosition, double initialVelocity, 
			double maxVelocity, double maxAcceleration) {
		this.currentPosition = initialPosition;
		this.finalPosition = finalPosition;
		// TODO may not work in reverse
		directionSign = this.finalPosition/finalPosition;
		this.currentVelocity = initialVelocity;
		this.maxVelocity = maxVelocity;
		//this.currentAcceleration = currentAcceleration;
		this.maxAcceleration = maxAcceleration;
		//this.totalTime = totalTime;
		
		// Save starting time
		startTime = System.currentTimeMillis();
		currentTime = startTime;
		
		Robot.log.writeLogEcho("Profile generator,init pos," + initialPosition + ",final pos," + finalPosition );
	}

	/**
	 * Call this method once per scheduler cycle.  This method calculates the distance that
	 * the robot should have traveled at this point in time, per the motion profile.
	 * @return Current target position for the robot, in inches
	 */
	public double getCurrentPosition(){
		long tempTime = System.currentTimeMillis();
		dt = ((double)(tempTime - currentTime))/1000.0;
		currentTime = tempTime;		
		
		double stoppingDistance = 0.5*currentVelocity*currentVelocity/maxAcceleration;
		if(finalPosition - currentPosition < stoppingDistance) currentAcceleration = -maxAcceleration;
		else if(currentVelocity < maxVelocity) currentAcceleration = maxAcceleration;
		else currentAcceleration = 0;
		
		currentVelocity = currentVelocity + currentAcceleration*dt;
		
		if(currentVelocity > maxVelocity) currentVelocity = maxVelocity;
		currentPosition = currentPosition + currentVelocity*dt;
		if(currentPosition > finalPosition) currentPosition = finalPosition;
		SmartDashboard.putNumber("Profile Position", currentPosition);
		SmartDashboard.putNumber("Profile Velocity", currentVelocity);
		Robot.log.writeLog("Profile generator,time since start," + getTimeSinceProfileStart() + ",dt," + dt +
				",current pos," + currentPosition*directionSign + ",current vel," + currentVelocity);
		return currentPosition*directionSign;
	}

	/**
	 * Returns the time since starting this profile generator
	 * @return
	 */
	public double getTimeSinceProfileStart() {
		return ((double)(currentTime - startTime))/1000.0;
	}
	
	/**
	 * Returns current velocity.  Don hacked this code, so please verify.
	 * @return Current velocity for profile calculation
	 */
	public double getCurrentVelocity(){
		// TODO Verify this code!
//		double stoppingDistance = 0.5*currentVelocity*currentVelocity/maxAcceleration;
//		if(finalPosition - currentPosition < stoppingDistance) currentAcceleration = -maxAcceleration;
//		else if(currentVelocity < maxVelocity) currentAcceleration = maxAcceleration;
//		else currentAcceleration = 0;
//		currentVelocity = currentVelocity + currentAcceleration*dt;
//		if(Math.abs(currentVelocity) > Math.abs(maxVelocity)) currentVelocity = maxVelocity;
//		currentPosition = currentPosition + currentVelocity*dt + .5*currentAcceleration*dt*dt;
//		if(Math.abs(currentPosition) > Math.abs(finalPosition)) currentPosition = finalPosition;
		return currentVelocity;
	}
}
