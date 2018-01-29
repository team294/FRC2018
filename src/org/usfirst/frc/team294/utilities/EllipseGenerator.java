package org.usfirst.frc.team294.utilities;

import org.usfirst.frc.team294.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EllipseGenerator {
	
	private double finalX = 0;
	private double finalY = 0;
	private double finalV = 0;
	private double finalA = 0;
	private double realX = 0;
	private double realY = 0;
	private double realV = 0;
	private double realA = 90;
	private double maxV = 10;
	private double maxA = 10;
	private double profileV = 0;
	private double profileA = 0;
	private double pArcLength = 0;
	private double currT = Math.PI;
	private double dt = 0.02;
	private double prevX = 0;
	private double prevY = 0;
	private double t2 = 0;
	private double A = 0;
	private double B = 0;
	private double profileX = 0;
	private double profileY = 0;

	//public double totalTime;
		
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
	public EllipseGenerator(double finalX, double finalY, double profileV, 
			double finalV, double finalA) {
		this.finalX = Math.abs(finalX);
		this.finalY = finalY;
		this.profileV = profileV;
		this.finalV = finalV;
		this.finalA = Math.toRadians(finalA);
		// TODO may not work in reverse
		directionSign = this.finalX/finalX;
		
		// Save starting time
		startTime = System.currentTimeMillis();
		currentTime = startTime;
		
		//Robot.log.writeLogEcho("Profile generator,init pos," + initialPosition + ",final pos," + finalPosition );
		
		double k = finalY/Math.tan(finalA);
		
		double t2  = Math.PI/2;
		if (Math.abs(finalA)>0.001)
		{
			t2 = 2*Math.atan(Math.sqrt(k)/Math.sqrt(-2*(this.finalX)+k));
		}
		
		if(t2 < 0 || t2 > Math.PI)
		{
			t2 = -t2;
		}
		this.B = finalY/Math.sin(t2);
		this.A = this.finalX/(1+Math.cos(t2));
		
		
	}

	/**
	 * Call this method once per scheduler cycle.  This method calculates the distance that
	 * the robot should have traveled at this point in time, per the motion profile.
	 * @return Current target position for the robot, in inches
	 */
	public void updateCurrentPosition(){
		long tempTime = System.currentTimeMillis();
		dt = ((double)(tempTime - currentTime))/1000.0;
		currentTime = tempTime;		
		
		//trapezoidal motion profile
		double stoppingDistance = 0.5*Math.pow(profileV,2)/maxA;
		double arcRemaining = getArc(currT,t2); // stopping distance found by arcLength check
		if(arcRemaining <= stoppingDistance && Math.abs(arcRemaining) > 0.01) profileA = -maxA;
		else if(profileV < maxV && Math.abs(arcRemaining)>0.01) profileA = maxA;
		else profileA = 0;
		double arcLength = pArcLength + profileV*dt + 0.5*profileA*Math.pow(dt, 2);
		double delta = arcLength - pArcLength;
		profileV += profileA*dt;
		currT = getT(currT,t2,delta); //use arc length that needs to be traveled to find next point on the ellipse
		pArcLength = arcLength; 
		profileX = A*(1+Math.cos(currT));
		profileY = B*Math.sin(currT);
		profileA = Math.atan((finalY-prevY)/(finalX-prevX))*180/Math.PI;
		prevX = profileX;
		prevY = profileY;
			
	}
	
	public  double getArc(double t1, double t2)
	{
		//Perform trapezoidal integration to get the arc length on the ellipse defined by a and b between t1 and t2
		double length = 0;
		//System.out.println("T1="+t1+",t2="+t2);
		double t = t1;
		double dt = 0.0001;
		while(Math.abs(t2-t)>dt)
		{
			length += Math.sqrt(Math.pow(A*Math.cos(t),2)+Math.pow(B*Math.sin(t), 2))*dt;
			//System.out.println("t="+t+",length="+length);
			t+=dt*Math.signum(t2-t1);
		}
		return length;
	}
	
	public double getT(double t1, double t2, double arcLength)
	{
		//Perform trapezoidal integration to find the t2 on the ellipse defined by a and b that produces the specified arcLength
		double fk = 0;
		double pfk = 0;
		double length = 0;
		double t = t1;
		double dt = 0.0001*Math.signum(t2-t1);
		while (length <= arcLength)
		{
			fk = Math.sqrt(Math.pow(A*Math.cos(t),2)+Math.pow(B*Math.sin(t), 2));
			length += (fk+pfk)/2*Math.abs(dt);
			t += dt;
			pfk = fk;
			//System.out.println("fk="+fk+",length="+length);
		}
		return t;	
	}
	
}
