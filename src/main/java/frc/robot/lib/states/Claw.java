// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.states;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

/** Class with methods related to the claw or color sensor */
public class Claw {
    
    private static ColorMatch colorMatcher;

    public Claw() {
        colorMatcher = new ColorMatch();
        RobotMap.clawMotor.setNeutralMode(NeutralMode.Brake);
        colorMatcher.addColorMatch(Constants.Claw.CUBE_COLOR);
        colorMatcher.addColorMatch(Constants.Claw.CONE_COLOR);
    }

    public Color updateDetectedColor() {
        return RobotMap.clawColorSensor.getColor();
    }

    public Color getColor() {
        ColorMatchResult match = colorMatcher.matchClosestColor(Robot.detectedColor);
        if (match.confidence > 0.9) {
            return match.color;
        }
        return null;
    }

    public void setClawOpen() {
        RobotMap.clawMotor.set(ControlMode.Position, 0.0);
    }

    public void setClawClosedCube() {
        RobotMap.clawMotor.set(ControlMode.Position, Constants.Claw.CLOSED_CUBE_POSITION);
    }

    public void setClawClosedCone() {
        RobotMap.clawMotor.set(ControlMode.Position, Constants.Claw.CLOSED_CONE_POSITION);
    }
}
