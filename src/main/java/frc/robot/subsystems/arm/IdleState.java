// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.*;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.State;
import frc.robot.lib.Transition;

/** Add your docs here. */
public class IdleState extends State {

    /* Error at frc.robot.lib.StateMachine.setCurrentState(StateMachine.java:40):
     Unhandled exception: java.lang.NullPointerException: Cannot assign field "state_machine_name"
      because "this.currentState" is null */

    private static XboxController manipulatorController = RobotMap.manipulatorController;
    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return manipulatorController.getAButton() && !RobotMap.arm.getClawSensor();
        }, ArmStateMachine.intakeState));
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton() || 
                   (RobotMap.arm.getBaseSensor() && RobotMap.arm.getClawSensor());
        }, ArmStateMachine.outtakeState));
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getBaseSensor() && !RobotMap.arm.getClawSensor();
        }, ArmStateMachine.baseState));
       
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "purple" && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.fHighPurple));
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "purple" && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.bHighPurple));
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "yellow" && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.fHighYellow));
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "yellow" && 
                    manipulatorController.getXButton();
        }, ArmStateMachine.bHighPurple));

        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "purple" && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.fMidPurple));
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "purple" && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.bMidPurple));
        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "yellow" && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.fMidYellow));
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) &&
                    RobotMap.arm.getColorSensor() == "yellow" && 
                    manipulatorController.getYButton();
        }, ArmStateMachine.bMidPurple));

        transitions.add(new Transition(() -> {
            return (manipulatorController.getRightTriggerAxis() > 0.25) && 
                    manipulatorController.getRightBumper();
        }, ArmStateMachine.fHybrid));
        transitions.add(new Transition(() -> {
            return (manipulatorController.getLeftTriggerAxis() > 0.25) && 
                    manipulatorController.getRightBumper();
        }, ArmStateMachine.bHybrid));
    }   
    
    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        
    }
}
