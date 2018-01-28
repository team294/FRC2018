package org.usfirst.frc.team294.utilities;

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
	
	public double threshold;
	
	private double directionSign;

	//Function to update equations for when we increment them, function to say hey we're done
	public ProfileGenerator(double initialPosition, double finalPosition, double initialVelocity, double maxVelocity, double maxAcceleration, double threshold) {
		this.currentPosition = initialPosition;
		this.finalPosition = finalPosition;
		// TODO may not work in reverse
		directionSign = this.finalPosition/finalPosition;
		this.currentVelocity = initialVelocity;
		this.maxVelocity = maxVelocity;
		//this.currentAcceleration = currentAcceleration;
		this.maxAcceleration = maxAcceleration;
		this.dt = 0.02;
		this.threshold = threshold;
		//this.totalTime = totalTime;
	}

	public double getCurrentPosition(){
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
		return currentPosition*directionSign;
	}

	public double getCurrentVelocity(){
		double stoppingDistance = 0.5*currentVelocity*currentVelocity/maxAcceleration;
		if(finalPosition - currentPosition < stoppingDistance) currentAcceleration = -maxAcceleration;
		else if(currentVelocity < maxVelocity) currentAcceleration = maxAcceleration;
		else currentAcceleration = 0;
		currentVelocity = currentVelocity + currentAcceleration*dt;
		if(Math.abs(currentVelocity) > Math.abs(maxVelocity)) currentVelocity = maxVelocity;
		currentPosition = currentPosition + currentVelocity*dt + .5*currentAcceleration*dt*dt;
		if(Math.abs(currentPosition) > Math.abs(finalPosition)) currentPosition = finalPosition;
		return currentVelocity;
	}
}
