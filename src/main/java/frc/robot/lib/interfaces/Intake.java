// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import frc.robot.Constants;
import frc.robot.RobotMap;

/** Class with methods related to the claw or color sensor */
public class Intake {
    private static ColorMatch colorMatcher;
    private static Color colorMatch = null;

    private static Timer colorTimer = new Timer();

    public Intake() {
        colorMatcher = new ColorMatch();

        colorMatcher.addColorMatch(Constants.INTAKE.CUBE_COLOR);
        colorMatcher.addColorMatch(Constants.INTAKE.CONE_COLOR);

        RobotMap.intakeMotor.configPeakOutputForward(1.0);
        RobotMap.intakeMotor.configPeakOutputReverse(-1.0);
        RobotMap.intakeMotor.setInverted(true);
        colorTimer.start();
    }

    public static Color getColor() {
        if (!colorTimer.hasElapsed(0.1))
            return colorMatch;
        colorTimer.restart();
        ColorMatchResult match = colorMatcher.matchClosestColor(RobotMap.intakeColorSensor.getColor());
        if (match.confidence > 0.85) {
            colorMatch = match.color;
        }
        else {
            colorMatch = null;
        }
        return colorMatch;
    }

    public void runIntake() {
        RobotMap.intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE.INTAKE_POWER);
    }

    public void runOuttake() {
        RobotMap.intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE.OUTTAKE_POWER);
    }

}