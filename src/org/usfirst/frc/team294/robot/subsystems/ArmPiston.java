package org.usfirst.frc.team294.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Subsystem controlling the two pistons on the arm
 */
public class ArmPiston extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private final DoubleSolenoid armPistonMajor = new DoubleSolenoid(RobotMap.pneumaticArmPistonMajorIn,
			RobotMap.pneumaticArmPistonMajorOut);
	private final Solenoid armPistonMinorSingle = new Solenoid(RobotMap.pneumaticArmPistonMinorOut);

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

	/**
	 * 
	 * @return true if the major piston is retracted (limit switch)
	 */
	private boolean getMajorRet() {
		return !majorLimitRetract.get();
	}

	/**
	 * 
	 * @return true if the major piston is extended (limit switch)
	 */
	private boolean getMajorExt() {
		return !majorLimitExtend.get();
	}

	/**
	 * 
	 * @return true if the minor piston is retracted (limit switch)
	 */
	private boolean getMinorRet() {
		return !minorLimitRetract.get();
	}

	/**
	 * 
	 * @return true if the minor piston is extended (limit switch)
	 */
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
				armPistonMinorSingle.set(true);
			if (position == RobotMap.PistonPositions.Retracted)
				armPistonMinorSingle.set(false);
	}

	/**
	 * returns position of major piston according to sensors
	 * 
	 * @return
	 */
	public PistonPositions getMajor() {
		switch (majorPosition) {
		case Extended:
			SmartDashboard.putString("Major Piston", "Extended");
			break;
		case Retracted:
			SmartDashboard.putString("Major Piston", "Retracted");
			break;
		case Moving:
			SmartDashboard.putString("Major Piston", "Moving");
			break;
		case Null:
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
		switch (minorPosition) {
		case Extended:
			SmartDashboard.putString("Minor Piston", "Extended");
			break;
		case Retracted:
			SmartDashboard.putString("Minor Piston", "Retracted");
			break;
		case Moving:
			SmartDashboard.putString("Minor Piston", "Moving");
			break;
		case Null:
			SmartDashboard.putString("Minor Piston", "Null");
			break;
		}
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
		getMajor();
		getMinor();
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

	/**
	 * Extends both pistons only if in the proper zone
	 * 
	 * @return true if either of the pistons are extended, false if both are retracted
	 */
	public boolean smartExtend() {
		ArmZones zone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		if (zone == ArmZones.Low) {
			setMajor(PistonPositions.Retracted);
			setMinor(PistonPositions.Extended);
			return true;
		} else if (zone == ArmZones.High) {
			setMajor(PistonPositions.Extended);
			setMinor(PistonPositions.Extended);
			return true;
		} else {
			setMajor(PistonPositions.Retracted);
			setMinor(PistonPositions.Retracted);
			return false;
		}
		// TODO: Update to reflect that minor piston does not break 16 in extension limit
	}

	/**
	 * Extends the major piston <b>only if</b> legal to do so (i.e. in high zone)
	 * 
	 * @return true if piston extended, false otherwise
	 */
	public boolean smartExtendMajor() {
		ArmZones zone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		if (zone == ArmZones.High) {
			setMajor(PistonPositions.Extended);
			return true;
		} else {
			setMajor(PistonPositions.Retracted);
			return false;
		}
	}

	/**
	 * Extends the minor piston <b>only if</b> legal to do so (i.e. in high or low
	 * zone)
	 * 
	 * @return true if piston extended, false otherwise
	 */
	public boolean smartExtendMinor() {
		ArmZones zone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		if (zone == ArmZones.High || zone == ArmZones.Low) {
			setMinor(PistonPositions.Extended);
			return true;
		} else {
			setMinor(PistonPositions.Retracted);
			return false;
		}
		
		// TODO: Update command to reflect 16 in extension rule (minor can almost always be extended)
	}

	/**
	 * Retracts both pistons
	 */
	public void smartRetract() {
		setMajor(PistonPositions.Retracted);
		setMinor(PistonPositions.Retracted);
	}

	public void periodic() {
		updateState();
		updateSmartDashboard();
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}
}