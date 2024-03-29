// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.StateMachine;
import frc.robot.subsystems.LED.LEDStateMachine;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class Arm {

    private static boolean timerStarted = false;
    private static Timer timer = new Timer();
    private static double shoulderTarget = 0.0;
    private static double elbowTarget = 0.0;

    public Arm() {
        RobotMap.rightShoulderMotor.setInverted(true);
        RobotMap.rightElbowMotor.setInverted(true);

        RobotMap.leftShoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.rightShoulderMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.leftElbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.rightElbowMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        RobotMap.leftShoulderMotor.configNominalOutputForward(0, 10);
        RobotMap.leftShoulderMotor.configNominalOutputReverse(0, 10);
        RobotMap.leftShoulderMotor.configPeakOutputForward(0.25, 10);
        RobotMap.leftShoulderMotor.configPeakOutputReverse(-0.2, 10);
        RobotMap.leftShoulderMotor.configNeutralDeadband(0.001, 10);

        RobotMap.rightShoulderMotor.configNominalOutputForward(0, 10);
        RobotMap.rightShoulderMotor.configNominalOutputReverse(0, 10);
        RobotMap.rightShoulderMotor.configPeakOutputForward(0.25, 10);
        RobotMap.rightShoulderMotor.configPeakOutputReverse(-0.2, 10);
        RobotMap.rightShoulderMotor.configNeutralDeadband(0.001, 10);

        RobotMap.leftElbowMotor.configNominalOutputForward(0, 10);
        RobotMap.leftElbowMotor.configNominalOutputReverse(0, 10);
        RobotMap.leftElbowMotor.configPeakOutputForward(0.17, 10);
        RobotMap.leftElbowMotor.configPeakOutputReverse(-0.25, 10);
        RobotMap.leftElbowMotor.configNeutralDeadband(0.001, 10);

        RobotMap.rightElbowMotor.configNominalOutputForward(0, 10);
        RobotMap.rightElbowMotor.configNominalOutputReverse(0, 10);
        RobotMap.rightElbowMotor.configPeakOutputForward(0.17, 10);
        RobotMap.rightElbowMotor.configPeakOutputReverse(-0.25, 10);
        RobotMap.rightElbowMotor.configNeutralDeadband(0.001, 10);

        RobotMap.leftShoulderMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.rightShoulderMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.leftElbowMotor.configAllowableClosedloopError(0, 0, 10);
        RobotMap.rightElbowMotor.configAllowableClosedloopError(0, 0, 10);
        
        RobotMap.leftShoulderMotor.configForwardSoftLimitEnable(true);
        RobotMap.rightShoulderMotor.configForwardSoftLimitEnable(true);
        RobotMap.leftElbowMotor.configForwardSoftLimitEnable(true);
        RobotMap.rightElbowMotor.configForwardSoftLimitEnable(true);

        RobotMap.leftShoulderMotor.configForwardSoftLimitThreshold(Constants.SHOULDER_MOTOR.F_TRAVEL_LIMIT);
        RobotMap.rightShoulderMotor.configForwardSoftLimitThreshold(Constants.SHOULDER_MOTOR.F_TRAVEL_LIMIT);
        RobotMap.leftElbowMotor.configForwardSoftLimitThreshold(Constants.ELBOW_MOTOR.F_TRAVEL_LIMIT);
        RobotMap.rightElbowMotor.configForwardSoftLimitThreshold(Constants.ELBOW_MOTOR.F_TRAVEL_LIMIT);

        RobotMap.leftShoulderMotor.configReverseSoftLimitThreshold(Constants.SHOULDER_MOTOR.B_TRAVEL_LIMIT);
        RobotMap.rightShoulderMotor.configReverseSoftLimitThreshold(Constants.SHOULDER_MOTOR.B_TRAVEL_LIMIT);
        RobotMap.leftElbowMotor.configReverseSoftLimitThreshold(Constants.ELBOW_MOTOR.B_TRAVEL_LIMIT);
        RobotMap.rightElbowMotor.configReverseSoftLimitThreshold(Constants.ELBOW_MOTOR.B_TRAVEL_LIMIT);

        RobotMap.rightShoulderMotor.set(ControlMode.Follower, Constants.CAN_IDS.LEFT_SHOULDER_MOTOR);
        RobotMap.rightElbowMotor.set(ControlMode.Follower, Constants.CAN_IDS.LEFT_ELBOW_MOTOR);
    }

    public void setIdle() {
        moveArmPosition(100.0, 0.0);
    }

    public void setElbowIdle() {
        // RobotMap.leftShoulderMotor.getSelectedSensorPosition();
        RobotMap.leftElbowMotor.set(TalonFXControlMode.Position, 0.0);
        elbowTarget = 0.0;
    }

    public void moveArmPosition(double shoulder, double elbow) {
        RobotMap.leftShoulderMotor.set(TalonFXControlMode.Position, shoulder);
        RobotMap.leftElbowMotor.set(TalonFXControlMode.Position, elbow);
        shoulderTarget = shoulder;
        elbowTarget = elbow;
    }

    // method to check if arm has arrived at its position
    public static Boolean getArrived(double allowance, double time) {
        if (Math.abs(RobotMap.leftShoulderMotor.getSelectedSensorPosition() - shoulderTarget) <= Math.abs(allowance) && 
            Math.abs(RobotMap.leftElbowMotor.getSelectedSensorPosition() - elbowTarget) <= Math.abs(allowance)) {

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

    public void manualEncoderFix(State prevState) {
        if (prevState == ArmStateMachine.idleState) {
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(0.0);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(0.0);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(0.0);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(0.0);
        } else if (prevState == ArmStateMachine.scoreHighState && RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState){
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.HIGH_SCORE_CONE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.HIGH_SCORE_CONE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.HIGH_SCORE_CONE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.HIGH_SCORE_CONE.ELBOW_POSITION);
        } else if (prevState == ArmStateMachine.scoreHighState && RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.cubeLEDState){
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.HIGH_SCORE_CUBE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.HIGH_SCORE_CUBE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.HIGH_SCORE_CUBE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.HIGH_SCORE_CUBE.ELBOW_POSITION);
        } else if (prevState == ArmStateMachine.scoreMidState && RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState){
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.MID_SCORE_CONE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.MID_SCORE_CONE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.MID_SCORE_CONE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.MID_SCORE_CONE.ELBOW_POSITION);
        } else if (prevState == ArmStateMachine.scoreMidState && RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.cubeLEDState){
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.MID_SCORE_CUBE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.MID_SCORE_CUBE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.MID_SCORE_CUBE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.MID_SCORE_CUBE.ELBOW_POSITION);
        } else if (prevState == ArmStateMachine.scoreLowState && RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState){
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.LOW_SCORE_CONE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.LOW_SCORE_CONE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.LOW_SCORE_CONE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.LOW_SCORE_CONE.ELBOW_POSITION);
        }  else if (prevState == ArmStateMachine.scoreLowState && RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.cubeLEDState){
            // the low scoring state position is the same for both cubes and cones
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.LOW_SCORE_CONE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.LOW_SCORE_CONE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.LOW_SCORE_CONE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.LOW_SCORE_CONE.ELBOW_POSITION);
        } else if (prevState == ArmStateMachine.groundPickupState && RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState){
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.GROUND_INTAKE_CONE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.GROUND_INTAKE_CONE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.GROUND_INTAKE_CONE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.GROUND_INTAKE_CONE.ELBOW_POSITION);
        } else if (prevState == ArmStateMachine.groundPickupState && RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.cubeLEDState){
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.GROUND_INTAKE_CUBE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.GROUND_INTAKE_CUBE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.GROUND_INTAKE_CUBE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.GROUND_INTAKE_CUBE.ELBOW_POSITION);
        } else if (prevState == ArmStateMachine.substationIntakeState){
            RobotMap.leftShoulderMotor.setSelectedSensorPosition(Constants.SUBSTATION_INTAKE.SHOULDER_POSITION);
            RobotMap.rightShoulderMotor.setSelectedSensorPosition(Constants.SUBSTATION_INTAKE.SHOULDER_POSITION);
            RobotMap.leftElbowMotor.setSelectedSensorPosition(Constants.SUBSTATION_INTAKE.ELBOW_POSITION);
            RobotMap.rightElbowMotor.setSelectedSensorPosition(Constants.SUBSTATION_INTAKE.ELBOW_POSITION);
        } 
    }
}
