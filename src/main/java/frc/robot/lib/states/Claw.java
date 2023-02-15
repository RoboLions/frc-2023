// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.states;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import frc.robot.Robot;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Claw {

    // TODO: positions
    double CLOSED_CUBE_POSITION = 0.0;
    double CLOSED_CONE_POSITION = 0.0;

    String colorString;

    public Claw() {
        RobotMap.clawMotor.setNeutralMode(NeutralMode.Brake);
        RobotMap.colorMatcher.addColorMatch(RobotMap.cubeColor);
        RobotMap.colorMatcher.addColorMatch(RobotMap.coneColor);
    }

    public Color updateDetectedColor() {
        return RobotMap.clawColorSensor.getColor();
    }

    public Color getColor() {
        ColorMatchResult match = RobotMap.colorMatcher.matchClosestColor(Robot.detectedColor);
        return match.color;
    }

    public void setClawOpen() {
        RobotMap.clawMotor.set(ControlMode.Position, 0.0);
    }

    public void setClawClosedCube() {
        RobotMap.clawMotor.set(ControlMode.Position, CLOSED_CUBE_POSITION);
    }

    public void setClawClosedCone() {
        RobotMap.clawMotor.set(ControlMode.Position, CLOSED_CONE_POSITION);
    }
}
