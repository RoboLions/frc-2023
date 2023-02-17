// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.states;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Arm {

    private Timer timer = new Timer();

    public Arm() {
        RobotMap.armFirstStageMotor.setNeutralMode(NeutralMode.Brake);
        RobotMap.armSecondStageMotor.setNeutralMode(NeutralMode.Brake);
        RobotMap.wristMotor.setNeutralMode(NeutralMode.Brake);

        RobotMap.armFirstStageMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.armSecondStageMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        RobotMap.armFirstStageMotor.configNominalOutputForward(0, 10);
        RobotMap.armFirstStageMotor.configNominalOutputReverse(0, 10);
        RobotMap.armFirstStageMotor.configPeakOutputForward(1, 10);
        RobotMap.armFirstStageMotor.configPeakOutputReverse(-1, 10);
        RobotMap.armFirstStageMotor.configNeutralDeadband(0.001, 10);

        RobotMap.armSecondStageMotor.configNominalOutputForward(0, 10);
        RobotMap.armSecondStageMotor.configNominalOutputReverse(0, 10);
        RobotMap.armSecondStageMotor.configPeakOutputForward(1, 10);
        RobotMap.armSecondStageMotor.configPeakOutputReverse(-1, 10);
        RobotMap.armSecondStageMotor.configNeutralDeadband(0.001, 10);

        RobotMap.wristMotor.configNominalOutputForward(0, 10);
        RobotMap.wristMotor.configNominalOutputReverse(0, 10);
        RobotMap.wristMotor.configPeakOutputForward(1, 10);
        RobotMap.wristMotor.configPeakOutputReverse(-1, 10);
        RobotMap.wristMotor.configNeutralDeadband(0.001, 10);

        RobotMap.armFirstStageMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.armSecondStageMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.wristMotor.configAllowableClosedloopError(0, 0, 10);
    }

    /*public boolean getBaseSensor() {
        return false;
    }*/

    public void setIdle() {
        RobotMap.armFirstStageMotor.set(TalonFXControlMode.Position, 0.0);
        RobotMap.armSecondStageMotor.set(TalonFXControlMode.Position, 0.0);
        RobotMap.wristMotor.set(TalonFXControlMode.Position, 0.0);
    }

    public void moveArmPosition(double firstStage, double secondStage, double wristMotor) {
        RobotMap.armFirstStageMotor.set(TalonFXControlMode.Position, firstStage);
        RobotMap.armSecondStageMotor.set(TalonFXControlMode.Position, secondStage);
        RobotMap.wristMotor.set(TalonFXControlMode.Position, wristMotor);
    }

    // method to check if arm has arrived at its position
    public boolean getArrived(double allowance, double time) {

        // TODO: test and change error allowance
        if (Math.abs(RobotMap.armFirstStageMotor.getClosedLoopError()) <= Math.abs(allowance) && 
            Math.abs(RobotMap.armSecondStageMotor.getClosedLoopError()) <= Math.abs(allowance) &&
            Math.abs(RobotMap.wristMotor.getClosedLoopError()) <= Math.abs(allowance)) { 
            timer.start();
            if (timer.hasElapsed(time)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }
        return false;
    }

    public boolean getClawClosed() {

        // assume claw is closed after some # of counts
        if (RobotMap.claw.getColor() == RobotMap.cubeColor) {
            timer.start();
            if (timer.hasElapsed(Constants.Claw.clawClosedCubeTime)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }

        if (RobotMap.claw.getColor() == RobotMap.coneColor) {
            timer.start();
            if (timer.hasElapsed(Constants.Claw.clawClosedConeTime)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }

        return false;
    }

    // assume claw is open after some # of counts
    public boolean getClawOpen() {
        if (!RobotMap.claw.getColor() == RobotMap.coneColor || 
            !RobotMap.claw.getColor() == RobotMap.cubeColor) {
            timer.start();
            if (timer.hasElapsed(Constants.Claw.clawClosedConeTime)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }
        return false;
    }

    public void resetEncoders() {
        RobotMap.armFirstStageMotor.setSelectedSensorPosition(0.0);
        RobotMap.armSecondStageMotor.setSelectedSensorPosition(0.0);
        RobotMap.wristMotor.setSelectedSensorPosition(0.0);
    }
}
