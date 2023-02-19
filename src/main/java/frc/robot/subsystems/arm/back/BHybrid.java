// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.back;

import frc.robot.Constants;
import frc.robot.lib.statemachine.State;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class BHybrid extends State {

    @Override
    public void build() {
        // return to idle automatically after scored
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));

        // transition to mid level purple
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MID_SCORE_BUTTON) &&
            (RobotMap.claw.getColor() == Constants.Claw.CUBE_COLOR);
        }, ArmStateMachine.bMidPurple));

        // transition to high level purple
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.HIGH_SCORE_BUTTON) && 
            (RobotMap.claw.getColor() == Constants.Claw.CUBE_COLOR);
        }, ArmStateMachine.bHighPurple));

        // transition to mid level yellow
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MID_SCORE_BUTTON) &&
            (RobotMap.claw.getColor() == Constants.Claw.CONE_COLOR);
        }, ArmStateMachine.bMidYellow));

        // transition to high level yellow
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.HIGH_SCORE_BUTTON) && 
            (RobotMap.claw.getColor() == Constants.Claw.CONE_COLOR);
        }, ArmStateMachine.bHighYellow));

        // return to idle manually
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.IDLE_BUTTON);
        }, ArmStateMachine.idleState));

        // transition to control arm manually
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MANUAL_MODE_BUTTON);
        }, ArmStateMachine.manualMoveState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.BHybrid.SHOULDER_POSITION, 
            Constants.BHybrid.ELBOW_POSITION, 
            Constants.BHybrid.WRIST_POSITION
        );
    }

    @Override
    public void execute() {
    }

    @Override
    public void exit() {
        
    }

}
