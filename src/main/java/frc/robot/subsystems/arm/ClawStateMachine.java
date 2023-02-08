// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.StateMachine;
import frc.robot.lib.Transition;

/** Add your docs here. */
public class ClawStateMachine extends StateMachine {

    public static ClawIdleState ClawidleState = new ClawIdleState();
    public static ClawIntakeState ClawintakeState = new ClawIntakeState();
    public static ClawOuttakeState ClawouttakeState = new ClawOuttakeState();
    public ClawStateMachine() {

        ClawidleState.build();
        ClawintakeState.build();
        ClawouttakeState.build();
        /*Supplier<Boolean> checkIdleButton = () -> {
            // TODO button, method
            return manipulatorController.getAButton();
        };

        Supplier<Boolean> checkPickupTransitions = () -> {
            // TODO button, methods
            // claw sensor must be false to pickup
            return RobotMap.arm.getBaseSensor() && !RobotMap.arm.getClawSensor();
        };

        Supplier<Boolean> checkOuttakeTransitions = () -> {
            // TODO button
            // base sensor must be true to outtake
            return manipulatorController.getXButton() && RobotMap.arm.getBaseSensor();
        };

        Supplier<Boolean> checkIntakeSubstationTransitions = () -> {
            // TODO button
            // claw sensor must be false to intake
            return manipulatorController.getYButton() && !RobotMap.arm.getClawSensor();
        };

        Supplier<Boolean> checkIntakeToIdleTransitions = () -> {
            // TODO button
            // claw sensor must be true to idle
            return manipulatorController.getAButton() || RobotMap.arm.getClawSensor();
        };

        Supplier<Boolean> checkHighPurpleFront = () -> {
            // TODO button, methods
            return manipulatorController.getRightTriggerAxis() > 0.25 && (RobotMap.arm.getColorSensor() == "purple") && manipulatorController.getBButton();
        };

        Supplier<Boolean> checkIdleTransitions = () -> {
            // TODO button
            // claw sensor must be false to idle
            return manipulatorController.getAButton() || !RobotMap.arm.getClawSensor();
        };

        Supplier<Boolean> checkHighPurpleBack = () -> {
            // TODO button, methods
            return manipulatorController.getLeftTriggerAxis() > 0.25 && (RobotMap.arm.getColorSensor() == "purple") && manipulatorController.getBButton();
        };

        Supplier<Boolean> checkMidPurpleBack = () -> {
            // TODO button, methods
            return manipulatorController.getLeftTriggerAxis() > 0.25 && (RobotMap.arm.getColorSensor() == "purple") && manipulatorController.getLeftBumper();
        };

        Supplier<Boolean> checkMidYellowBack = () -> {
            // TODO button, methods
            return manipulatorController.getLeftTriggerAxis() > 0.25 && (RobotMap.arm.getColorSensor() == "yellow") && manipulatorController.getLeftBumper();
        };

        Supplier<Boolean> checkHighYellowBack = () -> {
            // TODO button, methods
            return manipulatorController.getLeftTriggerAxis() > 0.25 && (RobotMap.arm.getColorSensor() == "yellow") && manipulatorController.getLeftBumper();
        };

        Supplier<Boolean> checkArrivedTrue = () -> {
            // TODO method
            return RobotMap.arm.checkArrived();
        };

        /*intakeState.addTransition(new Transition(checkIdleButton, idleState));
        intakeState.addTransition(new Transition(checkClawSensorFalse, idleState));
        idleState.addTransition(new Transition(checkIntakeButton, intakeState));
        outtakeState.addTransition(new Transition(checkArrivedTrue, idleState));
        dropState.addTransition(new Transition(checkClawSensorFalse, idleState));*/

        setCurrentState(ClawidleState);
    }
}
