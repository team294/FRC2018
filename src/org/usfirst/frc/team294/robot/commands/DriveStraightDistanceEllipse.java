package org.usfirst.frc.team294.robot.commands;

import java.util.function.Supplier;

import org.omg.PortableServer.ImplicitActivationPolicyOperations;
import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.utilities.ToleranceChecker;
import org.usfirst.frc.team294.utilities.EllipseGenerator;
import org.usfirst.frc.team294.utilities.ProfileGenerator;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Used for when you want to drive straight at some specified angle, uses motion profiling
 */
public class DriveStraightDistanceEllipse extends Command {

	private double distErr = 0;
//	private double distanceTravel;
	private double finalX, finalY;
	private double currentDistance;
	private ToleranceChecker tolCheck;
	private boolean success;
	private double distSpeedControl;
	
	private final double 
			kPdist = 0.05, 
			kDdist = 0.5,
			kIdist = 0.00; // not used
	
	private double prevDistErr;
	private double angleErr;
	private double intErr = 0;
	private double prevAngleErr;
	private double prevDistanceTicks;
	private double prevX;
	private double prevY;
	
	private double kPangle = 0.11;//.06;
	private double kDangle = .2;
	private double kIangle = .002;
	
	private double curve;
	private double minSpeed = .1;
	private double angleBase;
//	private ProfileGenerator trapezoid;
	private EllipseGenerator ellipse;
	
	public DriveStraightDistanceEllipse(double x, double y, double angleBase) {
		requires(Robot.driveTrain);
//		this.distanceTravel = distanceTravel;
		this.finalX = x;
		this.finalY = y;
		this.angleBase = angleBase;
	}

	public double encoderTicksToInches(double encoderticks) {
		return (encoderticks / RobotMap.encoderTicksPerRevolution) * RobotMap.wheelCircumference * RobotMap.driveTrainDistanceFudgeFactor;
	}

	public double inchesToEncoderTicks(double inches) {
		return (inches / RobotMap.wheelCircumference / RobotMap.driveTrainDistanceFudgeFactor) * RobotMap.encoderTicksPerRevolution;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
//		distanceTravel = SmartDashboard.getNumber("DistToTravelDSDG", 60);
		distErr = 0;
		prevDistErr = 0;
		angleErr = 0;
		success = false;
		distSpeedControl = 0;
		tolCheck = new ToleranceChecker(1, 5);
		Robot.driveTrain.zeroLeftEncoder();
		Robot.driveTrain.zeroRightEncoder();
//		trapezoid = new ProfileGenerator(0.0, distanceTravel, 0, 120, 120);
		prevDistanceTicks = 0;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double currentDistanceTicks = (Robot.driveTrain.getLeftEncoderPosition()+Robot.driveTrain.getRightEncoderPosition())/2;
		double currentDistanceInches = encoderTicksToInches(currentDistanceTicks);
		this.currentDistance = currentDistanceInches;
//		distErr = trapezoid.getCurrentPosition() - currentDistanceInches;
		distErr = ellipse.getDistErr();
		success = tolCheck.success(Math.abs(distErr));
		
		if (!success) {
			distSpeedControl = distErr * kPdist + (distErr - prevDistErr) * kDdist;
			prevDistErr = distErr;
			//distSpeedControl = distSpeedControl > percentPower ? percentPower : distSpeedControl;
			//distSpeedControl = distSpeedControl < -percentPower ? -percentPower : distSpeedControl;
			// TODO change this to max percent power, instead of scaling power
			if (distSpeedControl > 0) {
				distSpeedControl = (distSpeedControl < minSpeed) ? minSpeed : distSpeedControl;
			} else {
				distSpeedControl = (distSpeedControl > -minSpeed) ? -minSpeed : distSpeedControl;
			}
			angleErr = ellipse.getAngleErr();
			angleErr = (angleErr > 180) ? angleErr - 360 : angleErr;
			intErr = intErr + angleErr * 0.02;
			double dAngleErr = angleErr - prevAngleErr;
			prevAngleErr = angleErr;
			curve = angleErr * kPangle + intErr * kIangle + dAngleErr * kDangle;
			//curve = (curve > 0.5) ? 0.5 : curve;
			//curve = (curve < -0.5) ? -0.5 : curve;
			//curve = (distErr >= 0) ? curve : -curve;
			//Robot.driveTrain.driveAtCurve(distSpeedControl, curve);

			double diffTicks = currentDistanceTicks - prevDistanceTicks;
			double diffInches = this.encoderTicksToInches(diffTicks);
			
			Robot.driveTrain.addFieldPositionX(diffInches*Math.cos(Robot.driveTrain.getGyroRotation()));
			Robot.driveTrain.addFieldPositionY(diffInches*Math.sin(Robot.driveTrain.getGyroRotation()));
			
			prevDistanceTicks = currentDistanceTicks;
			this.prevX = Robot.driveTrain.getFieldPositionX();
			this.prevY = Robot.driveTrain.getFieldPositionY();
		}

//		SmartDashboard.putNumber("Distance Calculated", trapezoid.getCurrentPosition());
//		SmartDashboard.putNumber("Distance Error", distanceTravel - currentDistanceInches);
		SmartDashboard.putNumber("Actual Distance", currentDistance);
		SmartDashboard.putNumber("Actual Percent Power", distSpeedControl);
		SmartDashboard.putNumber("Field X", prevX);
		SmartDashboard.putNumber("Field Y", prevY);
		SmartDashboard.putNumber("profile x", ellipse.profileX);
		SmartDashboard.putNumber("Profile Y", ellipse.profileY );
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if(success) {
//			SmartDashboard.putNumber("fDistance Calculated", trapezoid.getCurrentPosition());
//			SmartDashboard.putNumber("fSet Distance", distanceTravel);
			SmartDashboard.putNumber("fActual Distance", currentDistance);
		}
		return success;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
