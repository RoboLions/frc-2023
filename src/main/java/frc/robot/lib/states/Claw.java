// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.states;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.RobotMap;

/** Add your docs here. */
public class Claw {

    // TODO: positions
    double closeCubePosition = 0.0;
    double closeConePosition = 0.0;

    public Claw() {
        RobotMap.claw.setNeutralMode(NeutralMode.Brake);
    }

    public void setClawOpen() {
        RobotMap.claw.set(ControlMode.Position, 0.0);
    }

    public void setClawClosedCube() {
        RobotMap.claw.set(ControlMode.Position, closeCubePosition);
    }

    public void setClawClosedCone() {
        RobotMap.claw.set(ControlMode.Position, closeConePosition);
    }
}
