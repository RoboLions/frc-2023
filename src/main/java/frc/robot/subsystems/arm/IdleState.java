// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class IdleState extends State {

    @Override
    public void build() {
        // intake from substation with arm at front of bot 
        // if intake button == T and claw sensor == F
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.SUB_INTAKE_FRONT) && 
            (RobotMap.claw.getColor() != RobotMap.coneColor || 
            RobotMap.claw.getColor() != RobotMap.cubeColor);
        }, ArmStateMachine.fIntakeState));

        // intake from substation with arm at back of bot 
        // if intake button == T and claw sensor == F
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.SUB_INTAKE_BACK) && 
            (RobotMap.claw.getColor() != RobotMap.coneColor || 
            RobotMap.claw.getColor() != RobotMap.cubeColor);
        }, ArmStateMachine.bIntakeState));

        // outtake if outtake button pressed
        transitions.add(new Transition(() -> {
            return (RobotMap.manipulatorController.getPOV() >= 135.0 || RobotMap.manipulatorController.getPOV() <= 225.0);
        }, ArmStateMachine.outtakeState));

        // pickup from back
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.GROUND_INTAKE_BACK);
        }, ArmStateMachine.bPickupState));

        // pickup from front
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.GROUND_INTAKE_FRONT);
        }, ArmStateMachine.fPickupState));

        // FHighPurple if bot is facing front (field relative), color sensor == purple, button (level to score: high)
        // TODO: field relative transitions
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.HIGH_SCORE_BUTTON);
        }, ArmStateMachine.fHighPurple));

        // BHighPurple if bot is facing back, color sensor == purple, button (level to score: high)
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.HIGH_SCORE_BUTTON);
        }, ArmStateMachine.bHighPurple));

        // FHighYellow if bot is facing front, color sensor == yellow, button (level to score: high)
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.coneColor && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.HIGH_SCORE_BUTTON);
        }, ArmStateMachine.fHighYellow));

        // BHighYellow if bot is facing back, color sensor == yellow, button (level to score: high)
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.coneColor && 
            RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.HIGH_SCORE_BUTTON);
        }, ArmStateMachine.bHighYellow));

        // FMidPurple if bot is facing front, color sensor == purple, button (level to score: mid)
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MID_SCORE_BUTTON);
        }, ArmStateMachine.fMidPurple));

        // BMidPurple if bot is facing back, color sensor == purple, button (level to score: mid)
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MID_SCORE_BUTTON);
        }, ArmStateMachine.bMidPurple));

        // FMidYellow if bot is facing front, color sensor == yellow, button (level to score: mid)
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.coneColor && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MID_SCORE_BUTTON);
        }, ArmStateMachine.fMidYellow));

        // BMidYellow if bot is facing back, color sensor == yellow, button (level to score: mid)
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.coneColor && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MID_SCORE_BUTTON);
        }, ArmStateMachine.bMidYellow));

        // FHybrid if bot is facing front, button (level to score: low)
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.LOW_SCORE_BUTTON);
        }, ArmStateMachine.fHybrid));

        // BHybrid if bot is facing back, button (level to score: low)
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.LOW_SCORE_BUTTON);
        }, ArmStateMachine.bHybrid));

        // transition to control arm manually
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MANUAL_MODE_BUTTON);
        }, ArmStateMachine.manualMoveState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.setIdle();
        RobotMap.arm.resetEncoders();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
        
    }
}
    
