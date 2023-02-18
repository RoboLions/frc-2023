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
        RobotMap.shoulderMotor.setNeutralMode(NeutralMode.Brake);
        RobotMap.elbowMotor.setNeutralMode(NeutralMode.Brake);
        RobotMap.wristMotor.setNeutralMode(NeutralMode.Brake);

        RobotMap.shoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        RobotMap.shoulderMotor.configNominalOutputForward(0, 10);
        RobotMap.shoulderMotor.configNominalOutputReverse(0, 10);
        RobotMap.shoulderMotor.configPeakOutputForward(1, 10);
        RobotMap.shoulderMotor.configPeakOutputReverse(-1, 10);
        RobotMap.shoulderMotor.configNeutralDeadband(0.001, 10);

        RobotMap.elbowMotor.configNominalOutputForward(0, 10);
        RobotMap.elbowMotor.configNominalOutputReverse(0, 10);
        RobotMap.elbowMotor.configPeakOutputForward(1, 10);
        RobotMap.elbowMotor.configPeakOutputReverse(-1, 10);
        RobotMap.elbowMotor.configNeutralDeadband(0.001, 10);

        RobotMap.wristMotor.configNominalOutputForward(0, 10);
        RobotMap.wristMotor.configNominalOutputReverse(0, 10);
        RobotMap.wristMotor.configPeakOutputForward(1, 10);
        RobotMap.wristMotor.configPeakOutputReverse(-1, 10);
        RobotMap.wristMotor.configNeutralDeadband(0.001, 10);

        RobotMap.shoulderMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.elbowMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.wristMotor.configAllowableClosedloopError(0, 0, 10);
    }

    /*public boolean getBaseSensor() {
        return false;
    }*/

    public void setIdle() {
        RobotMap.shoulderMotor.set(TalonFXControlMode.Position, 0.0);
        RobotMap.elbowMotor.set(TalonFXControlMode.Position, 0.0);
        RobotMap.wristMotor.set(TalonFXControlMode.Position, 0.0);
    }

    public void moveArmPosition(double shoulder, double elbow, double wristMotor) {
        RobotMap.shoulderMotor.set(TalonFXControlMode.Position, shoulder);
        RobotMap.elbowMotor.set(TalonFXControlMode.Position, elbow);
        RobotMap.wristMotor.set(TalonFXControlMode.Position, wristMotor);
    }

    // method to check if arm has arrived at its position
    public boolean getArrived(double allowance, double time) {

        // TODO: test and change error allowance
        if (Math.abs(RobotMap.shoulderMotor.getClosedLoopError()) <= Math.abs(allowance) && 
            Math.abs(RobotMap.elbowMotor.getClosedLoopError()) <= Math.abs(allowance) &&
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
        RobotMap.shoulderMotor.setSelectedSensorPosition(0.0);
        RobotMap.elbowMotor.setSelectedSensorPosition(0.0);
        RobotMap.wristMotor.setSelectedSensorPosition(0.0);
    }
}
