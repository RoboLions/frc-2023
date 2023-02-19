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
        // transitions.add(new Transition(() -> {
        //     return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.SUBSTATION_INTAKE_BUTTON) && 
        //         RobotMap.claw.getColor() == null;
        // }, ArmStateMachine.substationIntakeState));

        // outtake if outtake button pressed
        // transitions.add(new Transition(() -> {
        //     return (RobotMap.manipulatorController.getPOV() >= 135.0 || RobotMap.manipulatorController.getPOV() <= 225.0);
        // }, ArmStateMachine.outtakeState));

        // pickup from back
        // transitions.add(new Transition(() -> {
        //     return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.GROUND_INTAKE_BACK);
        // }, ArmStateMachine.bPickupState));

        // pickup from front
        // transitions.add(new Transition(() -> {
        //     return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.GROUND_INTAKE_FRONT);
        // }, ArmStateMachine.fPickupState));

        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() != null && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.HIGH_SCORE_BUTTON);
        }, ArmStateMachine.scoreHighState));
        
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() != null && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MID_SCORE_BUTTON);
        }, ArmStateMachine.scoreMidState));
        
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() != null && 
                    RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.LOW_SCORE_BUTTON);
        }, ArmStateMachine.scoreLowState));

        // transition to control arm manually
        // transitions.add(new Transition(() -> {
        //     return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.MANUAL_MODE_BUTTON);
        // }, ArmStateMachine.manualMoveState));
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
    
