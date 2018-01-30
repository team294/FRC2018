package org.usfirst.frc.team294.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.InterruptableSensorBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Subsystem controlling the two pistons on the arm
 */
public class ProtoArmPiston extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
			
	private final DoubleSolenoid armPistonMajor = new DoubleSolenoid(RobotMap.pneumaticArmPistonMajorIn,RobotMap.pneumaticArmPistonMajorOut);
	private final DoubleSolenoid armPistonMinor = new DoubleSolenoid(RobotMap.pneumaticArmPistonMinorIn, RobotMap.pneumaticArmPistonMinorOut);
	
	// Limit switches for pistons
	private final DigitalInput majorLimitRetract = new DigitalInput(RobotMap.majorPistonRetractedLimitSwitch);
	private final DigitalInput majorLimitExtend = new DigitalInput(RobotMap.majorPistonExtendedLimitSwitch);
	private final DigitalInput minorLimitRetract = new DigitalInput(RobotMap.minorPistonRetractedLimitSwitch);
	private final DigitalInput minorLimitExtend = new DigitalInput(RobotMap.minorPistonExtendedLimitSwitch);
	
	// Tracking piston position in software
	private PistonPositions majorPosition = PistonPositions.Null;
	private PistonPositions minorPosition = PistonPositions.Null;
	
	public ProtoArmPiston() {
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
	 * @param position only accepts PistonPositions.Extended and PistonPositions.Retracted 
	 * </br><b>Other values are ignored</b>
	 */
	public void setMajor(RobotMap.PistonPositions position) {
		if (position == RobotMap.PistonPositions.Extended) armPistonMajor.set(Value.kForward);
		if (position == RobotMap.PistonPositions.Retracted) armPistonMajor.set(Value.kReverse);
	}
	
	/**
	 * Sets the position of the minor piston
	 * @param position only accepts PistonPositions.Extended and PistonPositions.Retracted 
	 * </br><b>Other values are ignored</b>
	 */
	public void setMinor(RobotMap.PistonPositions position) {
		if (position == RobotMap.PistonPositions.Extended) armPistonMinor.set(Value.kForward);
		if (position == RobotMap.PistonPositions.Retracted) armPistonMinor.set(Value.kReverse);
	}
	
	public PistonPositions getMajor() {
		return majorPosition;
	}
	
	public PistonPositions getMinor() {
		return minorPosition;
	}
	
	/**
	 * Extends the major arm piston
	 */
	public void extendMajorPiston() {
		armPistonMajor.set(Value.kForward);
	}
	
	/**
	 * Retracts the major arm piston
	 */
	public void retractMajorPiston() {
		armPistonMajor.set(Value.kReverse);
	}
	
	/**
	 * Extends the minor arm piston
	 */
	public void extendMinorPiston() {
		armPistonMinor.set(Value.kForward);
	}
	
	/**
	 * Retracts the minor arm piston
	 */
	public void retractMinorPiston() {
		armPistonMinor.set(Value.kReverse);
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
	 * Updates the states of the piston. See RobotMap.PistonPositions for values.</br>
	 * Called every 20ms by the piston subsystem.
	 */
	public void updateState() {
		if (getMajorRet()) majorPosition = PistonPositions.Retracted;
		//else if (getMajorExt()) majorPosition = PistonPositions.Extended; //These should be switched once the hardware is in place, so priority is given to extended instead of retracted
		else majorPosition = PistonPositions.Moving;
		
		/*
		if (getMinorExt()) minorPosition = PistonPositions.Extended;
		else if (getMinorRet()) minorPosition = PistonPositions.Retracted;
		else minorPosition = PistonPositions.Moving;
		*/
		
		// Should we add a check to see if both limit switches are activated, and if so, set the position to null?
		// That would indicate faulty hardware, and we wouldn't want to move the arm into danger zones with faulty hardware
	}

	public void periodic() {
		updateSmartDashboard();
		updateState();
	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

