package org.usfirst.frc.team294.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem controlling the two pistons on the arm
 */
public class ArmPiston extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private final DoubleSolenoid armPistonMajor = new DoubleSolenoid(RobotMap.pneumaticArmPistonMajorIn,
			RobotMap.pneumaticArmPistonMajorOut);
//	private final DoubleSolenoid armPistonMinor = new DoubleSolenoid(RobotMap.pneumaticArmPistonMinorIn,
//			RobotMap.pneumaticArmPistonMinorOut);
	//TODO uncomment Minor Piston

	// Limit switches for pistons
	private final DigitalInput majorLimitRetract = new DigitalInput(RobotMap.majorPistonRetractedLimitSwitch);
	private final DigitalInput majorLimitExtend = new DigitalInput(RobotMap.majorPistonExtendedLimitSwitch);
	private final DigitalInput minorLimitRetract = new DigitalInput(RobotMap.minorPistonRetractedLimitSwitch);
	private final DigitalInput minorLimitExtend = new DigitalInput(RobotMap.minorPistonExtendedLimitSwitch);

	// Tracking piston position in software
	private PistonPositions majorPosition = PistonPositions.Null;
	private PistonPositions minorPosition = PistonPositions.Null;

	public ArmPiston() {
		super();
	}

	private boolean getMajorRet() {
		return !majorLimitRetract.get();
	}

	private boolean getMajorExt() {
		return !majorLimitExtend.get();
	}

	private boolean getMinorRet() {
		return !minorLimitRetract.get();
	}

	private boolean getMinorExt() {
		return !minorLimitExtend.get();
	}

	/**
	 * Sets the position of the major piston
	 * 
	 * @param position
	 *            only accepts PistonPositions.Extended and
	 *            PistonPositions.Retracted </br>
	 *            <b>Other values are ignored</b>
	 */
	public void setMajor(RobotMap.PistonPositions position) {
		if (position == RobotMap.PistonPositions.Extended)
			armPistonMajor.set(Value.kForward);
		if (position == RobotMap.PistonPositions.Retracted)
			armPistonMajor.set(Value.kReverse);
	}

	/**
	 * Sets the position of the minor piston
	 * 
	 * @param position
	 *            only accepts PistonPositions.Extended and
	 *            PistonPositions.Retracted </br>
	 *            <b>Other values are ignored</b>
	 */
	public void setMinor(RobotMap.PistonPositions position) {
		if (position == RobotMap.PistonPositions.Extended)
		//	armPistonMinor.set(Value.kForward);
		if (position == RobotMap.PistonPositions.Retracted);
		//	armPistonMinor.set(Value.kReverse);
	}//TODO uncomment Minor Piston

	/**
	 * returns position of major piston according to sensors
	 * 
	 * @return
	 */
	public PistonPositions getMajor() {
		switch (majorPosition) {
		case Extended : 
			SmartDashboard.putString("Major Piston", "Extended");
			break;
		case Retracted : 
			SmartDashboard.putString("Major Piston", "Retracted");
			break;
		case Moving : 
			SmartDashboard.putString("Major Piston", "Moving");
			break;
		case Null : 
			SmartDashboard.putString("Major Piston", "Null");
			break;
		}
		return majorPosition;
	}

	/**
	 * returns position of minor piston according to sensors
	 * 
	 * @return
	 */
	public PistonPositions getMinor() {
		return minorPosition;
	}

	/**
	 * Updates the SmartDashboard. Called every 20ms by the piston subsystem.
	 */
	public void updateSmartDashboard() {
		SmartDashboard.putBoolean("Major Extend", getMajorExt());
		SmartDashboard.putBoolean("Major Retract", getMajorRet());
		SmartDashboard.putBoolean("Minor Extend", getMinorExt());
		SmartDashboard.putBoolean("Minor Retract", getMinorRet());
	}

	/**
	 * Updates the states of the piston. See RobotMap.PistonPositions for
	 * values.</br>
	 * Called every 20ms by the piston subsystem.
	 */
	public void updateState() {
		if (getMajorExt())
			majorPosition = PistonPositions.Extended;
		else if (getMajorRet())
			majorPosition = PistonPositions.Retracted;
		else
			majorPosition = PistonPositions.Moving;

		if (getMinorExt())
			minorPosition = PistonPositions.Extended;
		else if (getMinorRet())
			minorPosition = PistonPositions.Retracted;
		else
			minorPosition = PistonPositions.Moving;

		// Should we add a check to see if both limit switches are activated, and if so,
		// set the position to null?
		// That would indicate faulty hardware, and we wouldn't want to move the arm
		// into danger zones with faulty hardware
	}

	public void smartExtend() {
		ArmZones zone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		if (zone == ArmZones.Low) {
			setMajor(PistonPositions.Retracted);
			setMinor(PistonPositions.Extended);
		} else if (zone == ArmZones.High) {
			setMajor(PistonPositions.Extended);
			setMinor(PistonPositions.Extended);
		} else {
			setMajor(PistonPositions.Retracted);
			setMinor(PistonPositions.Retracted);
		}
	}
	
	public void smartRetract() {
		setMajor(PistonPositions.Retracted);
		setMinor(PistonPositions.Retracted);
	}
	
	public void periodic() {
		updateSmartDashboard();
		updateState();
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}