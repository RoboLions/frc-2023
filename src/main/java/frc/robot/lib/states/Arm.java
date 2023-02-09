// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.states;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import frc.robot.RobotMap;

/** Add your docs here. */
public class Arm {

    public Arm() {}

    // TODO: get sensor
    public boolean getClawSensor() {
        return false;
    }

    public boolean getBaseSensor() {
        return false;
    }

    public String getColorSensor() {
        return "purple";
    }

    public void setIdle() {
        RobotMap.armFirstStage.set(TalonFXControlMode.Position, 0.0);
        RobotMap.armSecondStage.set(TalonFXControlMode.Position, 0.0);
        RobotMap.wrist.set(TalonFXControlMode.Position, 0.0);
    }

    public void moveArmPosition(double firstStage, double secondStage, double wrist) {
        RobotMap.armFirstStage.set(TalonFXControlMode.Position, firstStage);
        RobotMap.armSecondStage.set(TalonFXControlMode.Position, secondStage);
        RobotMap.wrist.set(TalonFXControlMode.Position, wrist);
    }
}
