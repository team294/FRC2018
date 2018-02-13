package org.usfirst.frc.team294.robot;

import org.usfirst.frc.team294.robot.OI.BottomKnob;
import org.usfirst.frc.team294.robot.OI.MiddleKnob;
import org.usfirst.frc.team294.robot.OI.TopKnob;
import org.usfirst.frc.team294.robot.commands.*;
import org.usfirst.frc.team294.robot.commands.ConveyorSetFromRobot.States;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team294.robot.commands.autoroutines.*;
import org.usfirst.frc.team294.robot.triggers.AxisTrigger;
import org.usfirst.frc.team294.robot.triggers.POVTrigger;
import org.usfirst.frc.team294.robot.commands.TurnGyro.Units;

public class OI {
	
	//Threshold position checker
	//For top and bottom knobs unless otherwise specified
	double[] knobThreshold=new double[]{-0.911,-0.731,-0.551,-0.367,-0.1835,-0.0035,0.1775,0.3605,0.5455,0.7285,0.91};
	//For middle knob only
	double[] middleKnobThreshold=new double[] {-0.751,-0.25,0.2525,0.7525};

	public enum TopKnob {
		minus6degrees,minus5degrees,minus4degrees,minus2degrees,minus3degrees,minus1degree,noChange,
		plus1degree, plus2degrees,plus3degrees,plus4degrees,plus5degrees
	}
	public enum MiddleKnob{
		PositionOne,PositionTwo,PositionThree,PositionFour,PositionFive
	}
	public enum BottomKnob {
		PositionOne, PositionTwo, PositionThree, PositionFour, PositionFive, PositionSix, PositionSeven, PositionEight, PositionNine, PositionTen, PositionEleven, PositionTwelve
	}
	TopKnob[] TopKnobPositions = new TopKnob[] {TopKnob.minus6degrees, TopKnob.minus5degrees, TopKnob.minus4degrees, TopKnob.minus3degrees,
			TopKnob.minus2degrees, TopKnob.minus1degree, TopKnob.noChange, TopKnob.plus1degree, TopKnob.plus2degrees, TopKnob.plus3degrees, TopKnob.plus4degrees,
			TopKnob.plus5degrees};
	MiddleKnob[] MiddleKnobPositions = new MiddleKnob[] {MiddleKnob.PositionOne, MiddleKnob.PositionTwo, MiddleKnob.PositionThree,MiddleKnob.PositionFour, MiddleKnob.PositionFive};
	BottomKnob[] BottomKnobPositions= new BottomKnob[] {BottomKnob.PositionOne, BottomKnob.PositionTwo, BottomKnob.PositionThree, BottomKnob.PositionFour, BottomKnob.PositionFive,
			BottomKnob.PositionSix, BottomKnob.PositionSeven, BottomKnob.PositionEight, BottomKnob.PositionNine, BottomKnob.PositionTen, BottomKnob.PositionEleven, BottomKnob.PositionTwelve
	};
	
	// Optional commands based on start position
	Command[] MiddleKnobCommands = new Command[] {
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand()
	};
	
	// Commands based on alliance abilities (auto routines)
	Command[] BottomKnobCommands = new Command[] {	
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
		//new AutoCommand(),
	};
	
	public Joystick leftJoystick = new Joystick(0); // Left Joystick is in port 0
	public Joystick rightJoystick = new Joystick(1); // Right Joystick is in port 1
	public Joystick coPanel = new Joystick(2);
	public Joystick xboxController = new Joystick(3);
	public Joystick armJoystick = new Joystick(4); // Arm Joystick is in port 4
	
	SendableChooser<Integer> chooser_autoPlan = new SendableChooser<>();
	SendableChooser<Integer> chooser_startPosition = new SendableChooser<>();

	public OI() {
		
		// Create button arrays for the input devices
		Button[] left = new Button[12];
	    Button[] right = new Button[12];
	    Button[] coP =  new Button[15];
	    Button[] xbB = new Button[11];
	    Trigger xbLT = new AxisTrigger(xboxController, 2, 0.9);
        Trigger xbRT = new AxisTrigger(xboxController, 3, 0.9);

        //TODO:  Make sure all controllers are set up to the correct commands. 
        // What does this mean? Test if the buttons call the commands indicated?

		Trigger xbPovUp = new POVTrigger(xboxController, 0);
        Trigger xbPovRight = new POVTrigger(xboxController, 90);
        Trigger xbPovDown = new POVTrigger(xboxController, 180);
        Trigger xbPovLeft = new POVTrigger(xboxController, 270);
        
        // Declare left and right joystick buttons
	    for (int i = 1; i < left.length; i++) {
	    	left[i] = new JoystickButton(leftJoystick, i);
	    	right[i] = new JoystickButton(rightJoystick, i);
	    	if (i == 1) {
	    		//right[i].whenPressed();  // Automatic cube pick up with vision
	    		//left[i].whenPressed();  // Automatic cube pick up with vision
	    	} else if (i == 3) {
	    		//right[i].whenPressed(new SwitchDriveDirection(true)); // Switch drive direction
	    		//left[i].whenPressed(new SwitchDriveDirection(false)); // Switch drive direction
	    	} else if (i == 2) {
	    		//right[i].whenPressed(); // Auto driving routines
	    		//left[i].whenPressed(); // Auto driving routines
	    	} else if (i == 4 || i == 5) {
	    		right[i].whenPressed(new DriveWithJoysticks());
	    		left[i].whenPressed(new DriveWithJoysticks());
	    	} else {
	    		right[i].whenPressed(new ShiftUp());
	    		left[i].whenPressed(new ShiftDown());
	    	}
	    }

	     // Declare codriver panel switches
	    for (int i = 1; i < coP.length; i++) {
	    	coP[i] = new JoystickButton(coPanel, i);
		}
	    
	    // Xbox controller buttons
	    for (int i = 1; i < xbB.length; i++) {
	    	xbB[i] = new JoystickButton(xboxController, i);
	    }
	    
	    // Bind commands to the codriver panel switches
	    //coP[1].whenPressed(new Command()); // Reserved for climbing sequences
	    //coP[2].whenPressed(new Command()); // Reserved for climbing sequences
	    //coP[3].whenPressed(new Command()); // Stop all flywheels
	    //coP[4].whenPressed(new Command()); // Prepare to score cube (rev up flywheels), alternate kill flywheels
	    //coP[5].whenPressed(new Command()); // Score cube
	    //coP[6].whenPressed(new Command()); // Intake mechanism up
	    //coP[7].whenPressed(new Command()); // Intake mechanism down
	    //coP[8].whenPressed(new Command()); // Arm to intake position
	    //coP[9].whenPressed(new PassiveOuttake()); // Outtake
	    //coP[10].whenPressed(new Command()); // Arm to backwards for scale
	    //coP[11].whenPressed(new LoadCubeSequence()); // Intake sequence
	    //coP[12].whenPressed(new Command()); // Toggle claw open/closed
	    //coP[13].whenPressed(new Command()); // Arm to switch position
	    //coP[14].whenPressed(new Command()); // Arm to alternate scale position
	    
	    // Xbox controller buttons
	    //xbB[1].whenPressed(new Command()); // Lower intake mechanism
	    //xbB[2].whenPressed(new PassiveOuttake()); // Outtake
	    //xbB[3].whenPressed(new LoadCubeSequence()); // Intake sequence
	    //xbB[4].whenPressed(new Command()); // Raise intake mechanism
	    //xbB[5].whenPressed(new Command()); // Open claw jaws
	    //xbB[6].whenPressed(new Command()); // Close claw jaws
	    //xbB[7].whenPressed(new Command()); // Reserved for climbing
	    //xbB[8].whenPressed(new Command()); // Reserved for climbing
	    //xbB[9].whenPressed(new Command()); // Stop flywheels
	    //xbB[10].whenPressed(new Command()); // Override climb OR arm
	    
	    //xbPovUp.whenActive(new Command()); // Arm to scale backwards
	    //xbPovDown.whenActive(new Command()); // Arm to intake position
	    //xbPovLeft.whenActive(new Command()); // Arm to switch position
	    //xbPovRight.whenActive(new Command()); // Arm to alternate scale position
	    
	    // Xbox triggers
	    //xbLT.whenActive(new Command()); // Prepare to score cube (rev up flywheels), alternate kill flywheels
	    //xbRT.whenActive(new Command()); // Score cube
		
		// Initialize our auto plan chooser
		chooser_autoPlan.addDefault("do Closest, if both far do scale", 0);
		chooser_autoPlan.addObject("do Closest, if both far do switch from front", 1); // Should never need this routine
		chooser_autoPlan.addObject("do closest, if both far do switch from back", 2);
		chooser_autoPlan.addObject("do Scale only", 3);
		chooser_autoPlan.addObject("do Switch only from middle", 4);
		chooser_autoPlan.addObject("Go to baseline", 5);
		SmartDashboard.putData("Auto Plan Selection", chooser_autoPlan);
		SmartDashboard.putData("AutoTest1",new AutoTest1());
		
		// Initialize our position chooser
		chooser_startPosition.addDefault("- choose from below -", 0);
		chooser_startPosition.addObject("Left", 1);
		chooser_startPosition.addObject("Middle", 2);
		chooser_startPosition.addObject("Right", 3);
		
		// Smart Dashboard Commands
		SmartDashboard.putData("Start Position Selection", chooser_startPosition);
	
		SmartDashboard.putData("Start Drive Train", new DriveWithJoysticks());
		
		SmartDashboard.putData("Retract Arm", new ArmRetract());
		
		SmartDashboard.putData("Extend Arm", new ArmExtend());
		
		SmartDashboard.putData("Control Arm Motor Joystick", new ArmMotorControl());  //TODO Fix me
				
		SmartDashboard.putData("Set Arm Position", new SetArmFromSmartDashboard());
		SmartDashboard.putNumber("set Arm Angle", 0);
		
		SmartDashboard.putData("Turn heckla small", new TurnGyro(90, Units.Degrees));
		SmartDashboard.putData("DriveStraightDistanceProfile", new DriveStraightDistanceProfile(24, 0));
		SmartDashboard.putNumber("DistToTravelDSDG", 150);
		SmartDashboard.putData(" ProfileTest", new DriveStraightDistanceEllipse(100, 1000, 0));
		
		SmartDashboard.putData("Photo Switch", new ReadPhotoSwitch()); // For use with the intake
		SmartDashboard.putData("Pick Up Cube", new CubePickUp());
		SmartDashboard.putData("Release Cube", new CubeLetGo());
		SmartDashboard.putData("Shoot Out Cube", new CubeShootOut());

		SmartDashboard.putData("Set Climb Motor to 50% forwards", new ClimbSetPercentPower(.50)); 
		SmartDashboard.putData("Set Climb Motor to 50% backwards", new ClimbSetPercentPower(-.50));
		SmartDashboard.putData("Stop Climb Motor", new ClimbSetPercentPower(0));
	}
	
	/**
	 * Reads the top knob.
	 * @return Raw position 0 (full ccw) to 11 (cw)
	 */
	public TopKnob readTopKnob() {
		double knobReading;
		int i=0;

		knobReading = coPanel.getRawAxis(4);
		int len=knobThreshold.length;
		for(i=0;i<len; i++) {
			if (knobReading<knobThreshold[i]) break;
		}
        
		if(i==len)return TopKnobPositions[len-1];
		return TopKnobPositions[i];
	}

	/**
	 * Reads the middle knob.
	 * @return Raw position 0 (full ccw) to 4 (cw).  Positions above 4 are indeterminate due to resistors missing.
	 */
	public int readMiddleKnobRaw() {
		double knobReading2;

		int i=0;
		knobReading2 = coPanel.getRawAxis(6);
		int len=middleKnobThreshold.length;
		for(i=0;i<len; i++) {
			if (knobReading2<middleKnobThreshold[i]) break;
		}

		return i;
	}
	
	/**
	 * Reads the middle knob.
	 * @return OI.MiddleKnob robot starting position constant for the current knob position
	 */
	public MiddleKnob readMiddleKnob(){
		return MiddleKnobPositions[readMiddleKnobRaw()];
	}

	/**
	 * Gets "drive, turn, and shoot" command based on the robot starting position, as
	 * per the middle knob setting. 
	 * @return  Command to turn and shoot
	 */
	public Command getMiddleKnobCommand() {
		int i;
		
		i = readMiddleKnobRaw();
		if (i<MiddleKnobCommands.length) {
			return MiddleKnobCommands[i];
		} else {
			return null;
		}			
	}

	/**
	 * Reads the bottom knob.
	 * @return Raw position 0 (full ccw) to 11 (full cw)
	 */
	public int readBottomKnobRaw() {
		double knobReading;
		int i=0;
		
		knobReading = coPanel.getRawAxis(3);
		int len=knobThreshold.length;
		for(i=0;i<len; i++) {
			if (knobReading<knobThreshold[i]) break;
		}
        
		if(i==len)return (len-1);
		return (i);
	}

	/**
	 * Reads the bottom knob.
	 * @return OI.BottomKnob barrier constant for the current knob position
	 */
	public BottomKnob readBottomKnob() {
		return BottomKnobPositions[readBottomKnobRaw()];
	}
	
	/**
	 * Gets autonomous command to run based on bottom knob setting
	 * @return Command to run
	 */
	public Command getAutonomousCommand() {
		return BottomKnobCommands[readBottomKnobRaw()];
	}
	
	public int readAutoPlan() {
		return chooser_autoPlan.getSelected();
	}
	
	public int readStartPosition() {
		return chooser_startPosition.getSelected();
	}
}
