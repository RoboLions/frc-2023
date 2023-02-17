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
//intake transition
        transitions.add(new Transition(() -> {
            return manipulatorController.getBackButton() && !RobotMap.arm.getClawSensor();
        }, ArmStateMachine.intakeState));
//outtake transition
        transitions.add(new Transition(() -> {
            return manipulatorController.getStartButton() || 
                   (RobotMap.arm.getBaseSensor() && RobotMap.arm.getClawSensor());
        }, ArmStateMachine.outtakeState));
//pickup transition
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getBaseSensor() && !RobotMap.arm.getClawSensor();
        }, ArmStateMachine.pickupState));
//front high purple transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "purple" && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.fHighPurple));
//back high purple transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "purple" && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.bHighPurple));
//front high yellow transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "yellow" && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.fHighYellow));
//back high purple transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "yellow" && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.bHighPurple));
//front mid purple transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "purple" && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.fMidPurple));
//back mid purple transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "purple" && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.bMidPurple));
//front mid yellow transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "yellow" && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.fMidYellow));
//back mid purple transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "yellow" && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.bMidPurple));
//front hybrid transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) && 
                    manipulatorController.getAButton();
        }, ArmStateMachine.fHybrid));
//back hybrid transition
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) && 
                    manipulatorController.getAButton();
        }, ArmStateMachine.bHybrid));
    }
  
    
    @Override
    public void init() {

    }

    @Override
    public void execute() {
        // RobotMap.arm.setIdle();
    }

    @Override
    public void exit() {
        
    }
}
    
