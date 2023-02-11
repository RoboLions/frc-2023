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

    public static double counter = 0.0;
    public static double counter2 = 0.0;
    public static double counter3 = 0.0;
    public static double counter4 = 0.0;

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

    /* getClosedLoopError(): Gets the closed-loop error. The units depend on which control mode is in use. 
    If closed-loop is seeking a target sensor position, closed-loop error is the difference 
    between target and current sensor value (in sensor units. 
    Example 4096 units per rotation for CTRE Mag Encoder). */

    public boolean getArrived(double allowance) {

        // TODO: test and change error allowance
        if (Math.abs(RobotMap.armFirstStage.getClosedLoopError()) <= Math.abs(allowance) && 
            Math.abs(RobotMap.armSecondStage.getClosedLoopError()) <= Math.abs(allowance) &&
            Math.abs(RobotMap.wrist.getClosedLoopError()) <= Math.abs(allowance)) { 
            counter++;
            if (counter > 15.0) {
                return true;
            }
        } else {
            counter = 0.0;
        }
        return false;
    }

    public boolean getClawClosed() {
        // TODO: change counts
        // assume claw is closed after some # of counts
        if (getColorSensor() == "purple") {
            counter2++;
            if (counter2 > 10.0) {
                return true;
            }
        } else {
            counter2 = 0.0;
        }

        if (getColorSensor() == "yellow") {
            counter3++;
            if (counter3 > 15.0) {
                return true;
            }
        } else {
            counter3 = 0.0;
        }

        return false;
    }

    // assume claw is open after some # of counts
    public boolean getClawOpen() {
        if (getColorSensor() != "purple" || getColorSensor() != "yellow") {
            counter4++;
            if (counter4 > 5.0) {
                return true;
            }
        } else {
            counter4 = 0.0;
        }
        return false;
    }
}
