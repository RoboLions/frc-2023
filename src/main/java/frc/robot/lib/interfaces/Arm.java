// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Arm {

    private Timer timer = new Timer();

    public Arm() {
        RobotMap.shoulderMotor.setNeutralMode(NeutralMode.Coast);
        RobotMap.elbowMotor.setNeutralMode(NeutralMode.Coast);
        RobotMap.wristMotor.setNeutralMode(NeutralMode.Coast);

        RobotMap.wristMotor.setInverted(true);

        RobotMap.shoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.elbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.wristMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        RobotMap.shoulderMotor.configNominalOutputForward(0, 10);
        RobotMap.shoulderMotor.configNominalOutputReverse(0, 10);
        RobotMap.shoulderMotor.configPeakOutputForward(1.0, 10);
        RobotMap.shoulderMotor.configPeakOutputReverse(-1.0, 10);
        RobotMap.shoulderMotor.configNeutralDeadband(0.001, 10);

        RobotMap.elbowMotor.configNominalOutputForward(0, 10);
        RobotMap.elbowMotor.configNominalOutputReverse(0, 10);
        RobotMap.elbowMotor.configPeakOutputForward(1.0, 10);
        RobotMap.elbowMotor.configPeakOutputReverse(-1.0, 10);
        RobotMap.elbowMotor.configNeutralDeadband(0.001, 10);

        RobotMap.wristMotor.configNominalOutputForward(0, 10);
        RobotMap.wristMotor.configNominalOutputReverse(0, 10);
        RobotMap.wristMotor.configPeakOutputForward(1.0, 10);
        RobotMap.wristMotor.configPeakOutputReverse(-1.0, 10);
        RobotMap.wristMotor.configNeutralDeadband(0.001, 10);

        RobotMap.shoulderMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.elbowMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.wristMotor.configAllowableClosedloopError(0, 0, 10);

        RobotMap.shoulderMotor.configForwardSoftLimitEnable(true);
        RobotMap.elbowMotor.configForwardSoftLimitEnable(true);
        RobotMap.wristMotor.configForwardSoftLimitEnable(true);

        RobotMap.shoulderMotor.configForwardSoftLimitThreshold(Constants.SHOULDER_MOTOR.TRAVEL_LIMIT);
        RobotMap.elbowMotor.configForwardSoftLimitThreshold(Constants.ELBOW_MOTOR.TRAVEL_LIMIT);
        RobotMap.wristMotor.configForwardSoftLimitThreshold(Constants.WRIST_MOTOR.TRAVEL_LIMIT);

        RobotMap.shoulderMotor.configReverseSoftLimitThreshold(-Constants.SHOULDER_MOTOR.TRAVEL_LIMIT);
        RobotMap.elbowMotor.configReverseSoftLimitThreshold(-Constants.ELBOW_MOTOR.TRAVEL_LIMIT);
        RobotMap.wristMotor.configReverseSoftLimitThreshold(-Constants.WRIST_MOTOR.TRAVEL_LIMIT);
    }

    public void setIdle() {
        moveArmPosition(0.0, 0.0, 0.0);
    }

    public void moveArmPosition(double shoulder, double elbow, double wrist) {
        RobotMap.shoulderMotor.set(TalonFXControlMode.Position, shoulder);
        RobotMap.elbowMotor.set(TalonFXControlMode.Position, elbow);
        RobotMap.wristMotor.set(TalonFXControlMode.Position, wrist);
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

        // assume claw is closed after some # of seconds
        if (RobotMap.claw.getColor() == Constants.CLAW.CUBE_COLOR) {
            timer.start();
            if (timer.hasElapsed(Constants.CLAW.TIME_CLOSE_ON_CUBE)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }

        if (RobotMap.claw.getColor() == Constants.CLAW.CONE_COLOR) {
            timer.start();
            if (timer.hasElapsed(Constants.CLAW.TIME_CLOSE_ON_CONE)) {
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

    public double applyDeadband(double armManualInput) {
        if (armManualInput > Constants.STICK_DEADBAND || armManualInput < -Constants.STICK_DEADBAND) {
            return armManualInput;
        }
        return 0.0;
    }

    public double getScoringDirectionModifier() {
        double current_rotation = RobotMap.swerve.getPose().getRotation().getDegrees();
        boolean heading_left = current_rotation > 90 && current_rotation < 270;
        double modifier = (!heading_left &&
                DriverStation.getAlliance() == DriverStation.Alliance.Red) ||
            (heading_left &&
                DriverStation.getAlliance() == DriverStation.Alliance.Blue) ?
                1.0 : -1.0;
        return modifier;
    }

    public double getSubstationDirectionModifier() {
        return -1.0 * getScoringDirectionModifier();
    }
}
