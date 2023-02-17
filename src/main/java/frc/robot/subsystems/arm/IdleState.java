// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class IdleState extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    @Override
    public void build() {
        // intake from substation if intake button == T and claw sensor == F
        transitions.add(new Transition(() -> {
            return manipulatorController.getAButton() && 
            (RobotMap.claw.getColor() != RobotMap.coneColor || 
            RobotMap.claw.getColor() != RobotMap.cubeColor);
        }, ArmStateMachine.intakeState));

        // outtake if claw sensor == T and base sensor == T
        /*transitions.add(new Transition(() -> {
            return RobotMap.arm.getBaseSensor() && RobotMap.arm.getClawSensor();
        }, ArmStateMachine.outtakeState));

        // pickup if claw sensor == F and base sensor == T
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getBaseSensor() && !RobotMap.arm.getClawSensor();
        }, ArmStateMachine.pickupState));*/

        // FHighPurple if RT == T (front), color sensor == purple, button (level to score: high)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.fHighPurple));

        // BHighPurple if LT == T (back), color sensor == purple, button (level to score: high)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.bHighPurple));

        // FHighYellow if RT == T (front), color sensor == yellow, button (level to score: high)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.claw.getColor() == RobotMap.coneColor && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.fHighYellow));

        // BHighYellow if LT == T (back), color sensor == yellow, button (level to score: high)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.claw.getColor() == RobotMap.coneColor && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.bHighYellow));

        // FMidPurple if RT == T (front), color sensor == purple, button (level to score: mid)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.fMidPurple));

        // BMidPurple if LT == T (back), color sensor == purple, button (level to score: mid)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.claw.getColor() == RobotMap.cubeColor && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.bMidPurple));

        // FMidYellow if RT == T (front), color sensor == yellow, button (level to score: mid)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.claw.getColor() == RobotMap.coneColor && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.fMidYellow));

        // BMidPurple if LT == T (back), color sensor == yellow, button (level to score: mid)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.claw.getColor() == RobotMap.coneColor && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.bMidYellow));

        // FHybrid if RT == T (front), button (level to score: low)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) && 
                    manipulatorController.getBButton();
        }, ArmStateMachine.fHybrid));

        // BHybrid if LT == T (back), button (level to score: low)
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) && 
                    manipulatorController.getBButton();
        }, ArmStateMachine.bHybrid));

        // Manual control of arm
        transitions.add(new Transition(() -> {
            return manipulatorController.getLeftBumper();
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
    
