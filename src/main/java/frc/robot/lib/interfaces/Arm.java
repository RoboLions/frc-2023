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

    private boolean timerStarted = false;
    private Timer timer = new Timer();

    public Arm() {
        RobotMap.leftShoulderMotor.setNeutralMode(NeutralMode.Coast);
        RobotMap.rightShoulderMotor.setNeutralMode(NeutralMode.Coast);
        RobotMap.leftElbowMotor.setNeutralMode(NeutralMode.Coast);
        RobotMap.rightElbowMotor.setNeutralMode(NeutralMode.Coast);

        RobotMap.leftShoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.rightShoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.leftElbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.rightElbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        RobotMap.leftShoulderMotor.configNominalOutputForward(0, 10);
        RobotMap.leftShoulderMotor.configNominalOutputReverse(0, 10);
        RobotMap.leftShoulderMotor.configPeakOutputForward(1.0, 10);
        RobotMap.leftShoulderMotor.configPeakOutputReverse(-1.0, 10);
        RobotMap.leftShoulderMotor.configNeutralDeadband(0.001, 10);

        RobotMap.rightShoulderMotor.configNominalOutputForward(0, 10);
        RobotMap.rightShoulderMotor.configNominalOutputReverse(0, 10);
        RobotMap.rightShoulderMotor.configPeakOutputForward(1.0, 10);
        RobotMap.rightShoulderMotor.configPeakOutputReverse(-1.0, 10);
        RobotMap.rightShoulderMotor.configNeutralDeadband(0.001, 10);

        RobotMap.leftElbowMotor.configNominalOutputForward(0, 10);
        RobotMap.leftElbowMotor.configNominalOutputReverse(0, 10);
        RobotMap.leftElbowMotor.configPeakOutputForward(1.0, 10);
        RobotMap.leftElbowMotor.configPeakOutputReverse(-1.0, 10);
        RobotMap.leftElbowMotor.configNeutralDeadband(0.001, 10);

        RobotMap.rightElbowMotor.configNominalOutputForward(0, 10);
        RobotMap.rightElbowMotor.configNominalOutputReverse(0, 10);
        RobotMap.rightElbowMotor.configPeakOutputForward(1.0, 10);
        RobotMap.rightElbowMotor.configPeakOutputReverse(-1.0, 10);
        RobotMap.rightElbowMotor.configNeutralDeadband(0.001, 10);

        RobotMap.leftShoulderMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.rightShoulderMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.leftElbowMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.rightElbowMotor.configAllowableClosedloopError(0, 0, 10);
        
        RobotMap.leftShoulderMotor.configForwardSoftLimitEnable(true);
        RobotMap.rightShoulderMotor.configForwardSoftLimitEnable(true);
        RobotMap.leftElbowMotor.configForwardSoftLimitEnable(true);
        RobotMap.rightElbowMotor.configForwardSoftLimitEnable(true);

        RobotMap.leftShoulderMotor.configForwardSoftLimitThreshold(Constants.LEFT_SHOULDER_MOTOR.TRAVEL_LIMIT);
        RobotMap.rightShoulderMotor.configForwardSoftLimitThreshold(Constants.RIGHT_SHOULDER_MOTOR.TRAVEL_LIMIT);
        RobotMap.leftElbowMotor.configForwardSoftLimitThreshold(Constants.LEFT_ELBOW_MOTOR.TRAVEL_LIMIT);
        RobotMap.rightElbowMotor.configForwardSoftLimitThreshold(Constants.RIGHT_ELBOW_MOTOR.TRAVEL_LIMIT);

        RobotMap.leftShoulderMotor.configReverseSoftLimitThreshold(-Constants.LEFT_SHOULDER_MOTOR.TRAVEL_LIMIT);
        RobotMap.rightShoulderMotor.configReverseSoftLimitThreshold(-Constants.RIGHT_SHOULDER_MOTOR.TRAVEL_LIMIT);
        RobotMap.leftElbowMotor.configReverseSoftLimitThreshold(-Constants.LEFT_ELBOW_MOTOR.TRAVEL_LIMIT);
        RobotMap.rightElbowMotor.configReverseSoftLimitThreshold(-Constants.RIGHT_ELBOW_MOTOR.TRAVEL_LIMIT);
    }

    public void setIdle() {
        moveArmPosition(0.0, 0.0, 0.0, 0.0);
    }

    public void moveArmPosition(double leftShoulder, double rightShoulder, double leftElbow, double rightElbow) {
        RobotMap.leftShoulderMotor.set(TalonFXControlMode.Position, leftShoulder);
        RobotMap.rightShoulderMotor.set(TalonFXControlMode.Position, rightShoulder);
        RobotMap.leftElbowMotor.set(TalonFXControlMode.Position, leftElbow);
        RobotMap.rightElbowMotor.set(TalonFXControlMode.Position, rightElbow);
    }

    // method to check if arm has arrived at its position
    public Boolean getArrived(double allowance, double time) {

        // TODO: test and change error allowance
        if (Math.abs(RobotMap.leftShoulderMotor.getClosedLoopError()) <= Math.abs(allowance) && 
            Math.abs(RobotMap.rightShoulderMotor.getClosedLoopError()) <= Math.abs(allowance) &&
            Math.abs(RobotMap.leftElbowMotor.getClosedLoopError()) <= Math.abs(allowance) &&
            Math.abs(RobotMap.rightElbowMotor.getClosedLoopError()) <= Math.abs(allowance)) { 

            if (!timerStarted) {
                timer.start();
                timerStarted = true;
            }

            if (timer.hasElapsed(time)) {
                timer.stop();
                timer.reset();
                timerStarted = false;
                return true;
            }
            
            return false;
        }

        timer.stop();
        timer.reset();
        timerStarted = false;
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
        RobotMap.leftShoulderMotor.setSelectedSensorPosition(0.0);
        RobotMap.rightShoulderMotor.setSelectedSensorPosition(0.0);
        RobotMap.leftElbowMotor.setSelectedSensorPosition(0.0);
        RobotMap.rightElbowMotor.setSelectedSensorPosition(0.0);
    }

    public double applyDeadband(double armManualInput) {
        if (armManualInput > Constants.STICK_DEADBAND || armManualInput < -Constants.STICK_DEADBAND) {
            return armManualInput;
        }
        return 0.0;
    }

    // TODO: get correct limit switch values and uncomment when ready
    /*public int getRightLimitSwitchValue() {
        return RobotMap.rightShoulderMotor.getSensorCollection().isFwdLimitSwitchClosed();
    }

    public int getLeftLimitSwitchValue() {
        return RobotMap.leftShoulderMotor.getSensorCollection().isRevLimitSwitchClosed();
    }*/
}
