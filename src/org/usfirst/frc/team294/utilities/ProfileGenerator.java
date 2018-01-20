package org.usfirst.frc.team294.utilities;


public class ProfileGenerator {

	public double currentPosition;
	public double finalPosition;
	public double currentVelocity;
	public double maxVelocity;
	public double currentAcceleration;
	public double maxAcceleration;
	public double threshold;
	public double directionSign;
	public double dt;
	
	public ProfileGenerator(double currentPosition, double finalPosition, double currentVelocity, double maxVelocity, double currentAcceleration, double maxAcceleration, double threshold) {
		
		this.currentPosition = currentPosition;
		this.finalPosition = Math.abs(finalPosition);
		this.directionSign = (this.finalPosition)/finalPosition;
		this.currentVelocity = currentVelocity;
		this.maxVelocity = maxVelocity;
		this.currentAcceleration = currentAcceleration;
		this.maxAcceleration = maxAcceleration;
		this.threshold = threshold;
		this.dt = .02;
		
	}
	
	//
	public double getAcceleration() {
		
		double stoppingDistance = 0.5*currentVelocity*currentVelocity/maxAcceleration;
		if(stoppingDistance > (finalPosition-currentPosition)) {
			currentAcceleration = maxAcceleration;
		}
		else if (currentVelocity < maxVelocity){
			currentAcceleration = maxAcceleration;
		}
		if(currentVelocity > maxVelocity) {
			currentVelocity = maxVelocity;
		}
		else{
			currentAcceleration = 0;
		}
		
		currentVelocity = currentVelocity + currentAcceleration*dt;
		
		if(currentVelocity > maxVelocity) {
			currentVelocity = maxVelocity;
		}
		
		currentPosition = currentPosition + currentVelocity*dt + .5*currentAcceleration*dt*dt;

		if(currentPosition > finalPosition) {
			currentPosition = finalPosition;
		}
		
		return currentPosition*directionSign;
	}
	
}
