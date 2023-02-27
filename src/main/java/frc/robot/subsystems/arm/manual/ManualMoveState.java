// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.manual;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.ManipulatorControls;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class ManualMoveState extends State {

    @Override
    public void build() {
        // idle if idle button
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.idleState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.HIGH_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CUBE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreHighCubeState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MID_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CUBE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreMidCubeState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.LOW_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CUBE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreLowCubeState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.HIGH_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CONE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreHighConeState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MID_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CONE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreMidConeState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.LOW_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CONE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreLowConeState));
    }

    @Override
    public void init() {
    }

    @Override
    public void execute() {
        double elbowInput = RobotMap.manipulatorController.getRawAxis(ManipulatorControls.ELBOW_AXIS);
        RobotMap.leftElbowMotor.set(ControlMode.PercentOutput, RobotMap.arm.applyDeadband(elbowInput));

        double shoulderInput = RobotMap.manipulatorController.getRawAxis(ManipulatorControls.SHOULDER_AXIS);
        RobotMap.leftShoulderMotor.set(ControlMode.PercentOutput, RobotMap.arm.applyDeadband(-shoulderInput));
    }

    @Override
    public void exit() {
        
    }
}
