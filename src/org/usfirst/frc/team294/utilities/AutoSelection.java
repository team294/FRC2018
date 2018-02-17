package org.usfirst.frc.team294.utilities;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath1_SameSideScale;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath2_OppositeSideScale;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath3_SameSideSwitch;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath4_OppositeSideSwitchBack;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath5_SwitchFromMiddle;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath6_OppositeSideSwitchFront;
import org.usfirst.frc.team294.robot.commands.autoroutines.AutoPath7_Baseline;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelection {
	private boolean allianceSwitchLeft = false;
	private boolean scaleLeft = false;
	private boolean opponentSwitchLeft = false;

	public Command autonomousCommand;

	public AutoSelection() {
		super();
	}
	
	public void readGameData() {
		String gameData = DriverStation.getInstance().getGameSpecificMessage();

		if (gameData.charAt(0) == 'L') {
			SmartDashboard.putBoolean("Close Switch Left", true);
			SmartDashboard.putBoolean("Close Switch Right", false);
			allianceSwitchLeft = true;
		} else {
			SmartDashboard.putBoolean("Close Switch Right", true);
			SmartDashboard.putBoolean("Close Switch Left", false);
			allianceSwitchLeft = false;
		}

		if (gameData.charAt(1) == 'L') {
			SmartDashboard.putBoolean("Scale Left", true);
			SmartDashboard.putBoolean("Scale Right", false);
			scaleLeft = true;
		} else {
			SmartDashboard.putBoolean("Scale Right", true);
			SmartDashboard.putBoolean("Scale Left", false);
			scaleLeft = false;
		}

		if (gameData.charAt(2) == 'L') {
			SmartDashboard.putBoolean("Far Switch Left", true);
			SmartDashboard.putBoolean("Far Switch Right", false);
			opponentSwitchLeft = true;
		} else {
			SmartDashboard.putBoolean("Far Switch Right", true);
			SmartDashboard.putBoolean("Far Switch Left", false);
			opponentSwitchLeft = false;
		}

		int fieldLayout, autoPlan;

		if (gameData.startsWith("LL"))
			fieldLayout = RobotMap.AutoFieldLayout.LL.ordinal();
		else if (gameData.startsWith("LR"))
			fieldLayout = RobotMap.AutoFieldLayout.LR.ordinal();
		else if (gameData.startsWith("RL"))
			fieldLayout = RobotMap.AutoFieldLayout.RL.ordinal();
		else
			fieldLayout = RobotMap.AutoFieldLayout.RR.ordinal();

		int programSelected;
		autoPlan = Robot.oi.readAutoPlan();
		int startPosition = Robot.oi.readStartPosition();

		if (startPosition == 0) {
			programSelected = RobotMap.startingLeftAutoPrograms[autoPlan][fieldLayout];
		} else if (startPosition == 1) {
			programSelected = RobotMap.startingMiddleAutoPrograms[autoPlan][fieldLayout];
		} else {
			programSelected = RobotMap.startingRightAutoPrograms[autoPlan][fieldLayout];

		}

		switch (programSelected) {
		case 1:
			autonomousCommand = new AutoPath1_SameSideScale(startPosition);
			Robot.log.writeLogEcho("Ran Auto Path 1 (same side scale), side = " + startPosition);
			break;
		case 2:
			autonomousCommand = new AutoPath2_OppositeSideScale(startPosition);
			Robot.log.writeLogEcho("Ran Auto Path 2 (opposite side scale), side = " + startPosition);
			break;
		case 3:
			autonomousCommand = new AutoPath3_SameSideSwitch(startPosition);
			Robot.log.writeLogEcho("Ran Auto Path 3 (same side switch), side = " + startPosition);
			break;
		case 4:
			autonomousCommand = new AutoPath4_OppositeSideSwitchBack(startPosition);
			Robot.log.writeLogEcho("Ran Auto Path 4 (opposite side switch back), side = " + startPosition);
			break;
		case 5:
			autonomousCommand = new AutoPath5_SwitchFromMiddle(allianceSwitchLeft);
			Robot.log.writeLogEcho("Ran Auto Path 5 (switch from middle), left = " + allianceSwitchLeft);
			break;
		case 6:
			autonomousCommand = new AutoPath6_OppositeSideSwitchFront(startPosition);
			Robot.log.writeLogEcho("Ran Auto Path 6 (opposite side switch front), side = " + startPosition);
			break;
		case 7:
			autonomousCommand = new AutoPath7_Baseline(startPosition);
			Robot.log.writeLogEcho("Ran Auto Path 7 (Go to baseline), side = " + startPosition);
			break;
		}
		
		SmartDashboard.putString("Auto path", autonomousCommand.getName());
		SmartDashboard.putNumber("Auto program #", programSelected);
		SmartDashboard.putNumber("Auto field selection", fieldLayout);
		SmartDashboard.putNumber("Auto plan selected", autoPlan);
		SmartDashboard.putNumber("Auto start position", startPosition);
		
		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();

		if (color == DriverStation.Alliance.Blue) {
			SmartDashboard.putBoolean("Alliance Color", true);
		} else {
			SmartDashboard.putBoolean("Alliance Color", false);
		}
	}
}
