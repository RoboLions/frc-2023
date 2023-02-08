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
            return manipulatorController.getBButton();
        }, ArmStateMachine.dropState));
        transitions.add(new Transition(() -> {
            return manipulatorController.getXButton() && RobotMap.arm.getColorSensor() == "Purple" && manipulatorController.getRightTriggerAxis() > 0.25;
        }, ArmStateMachine.lowPurple));
        transitions.add(new Transition(() -> {
            return manipulatorController.getXButton() && RobotMap.arm.getColorSensor() == "Yellow" && manipulatorController.getRightTriggerAxis() > 0.25;
        }, ArmStateMachine.lowYellow));
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
