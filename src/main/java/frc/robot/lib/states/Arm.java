// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.states;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import frc.robot.RobotMap;

/** Add your docs here. */
public class Arm {

    public Arm() {

        RobotMap.armFirstStage.setNeutralMode(NeutralMode.Brake);
        RobotMap.armSecondStage.setNeutralMode(NeutralMode.Brake);
        RobotMap.wrist.setNeutralMode(NeutralMode.Brake);

        RobotMap.armFirstStage.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.armSecondStage.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.wrist.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        RobotMap.armFirstStage.configNominalOutputForward(0, 10);
        RobotMap.armFirstStage.configNominalOutputReverse(0, 10);
        RobotMap.armFirstStage.configPeakOutputForward(1, 10);
        RobotMap.armFirstStage.configPeakOutputReverse(-1, 10);
        RobotMap.armFirstStage.configNeutralDeadband(0.001, 10);
        
        RobotMap.armSecondStage.configNominalOutputForward(0, 10);
        RobotMap.armSecondStage.configNominalOutputReverse(0, 10);
        RobotMap.armSecondStage.configPeakOutputForward(1, 10);
        RobotMap.armSecondStage.configPeakOutputReverse(-1, 10);
        RobotMap.armSecondStage.configNeutralDeadband(0.001, 10);

        RobotMap.wrist.configNominalOutputForward(0, 10);
        RobotMap.wrist.configNominalOutputReverse(0, 10);
        RobotMap.wrist.configPeakOutputForward(1, 10);
        RobotMap.wrist.configPeakOutputReverse(-1, 10);
        RobotMap.wrist.configNeutralDeadband(0.001, 10);

        RobotMap.armFirstStage.configAllowableClosedloopError(0, 0, 10);
        RobotMap.armSecondStage.configAllowableClosedloopError(0, 0, 10);
        RobotMap.wrist.configAllowableClosedloopError(0, 0, 10);
    }

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

    public void resetEncoders() {
        RobotMap.armFirstStage.setSelectedSensorPosition(0.0);
        RobotMap.armSecondStage.setSelectedSensorPosition(0.0);
        RobotMap.wrist.setSelectedSensorPosition(0.0);
    }
}
