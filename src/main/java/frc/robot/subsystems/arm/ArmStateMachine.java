// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.StateMachine;
import frc.robot.lib.Transition;
import frc.robot.subsystems.arm.back.BHighPurple;
import frc.robot.subsystems.arm.back.BMidPurple;
import frc.robot.subsystems.arm.back.BHybrid;
import frc.robot.subsystems.arm.back.BHighYellow;
import frc.robot.subsystems.arm.back.BMidYellow;
import frc.robot.subsystems.arm.front.FHighPurple;
import frc.robot.subsystems.arm.front.FMidPurple;
import frc.robot.subsystems.arm.front.FHybrid;
import frc.robot.subsystems.arm.front.FHighYellow;
import frc.robot.subsystems.arm.front.FMidYellow;
/** Add your docs here. */
public class ArmStateMachine extends StateMachine {

    public static IdleState idleState = new IdleState();
    public static IntakeState intakeState = new IntakeState();
    public static OuttakeState outtakeState = new OuttakeState();
    public static DropState dropState = new DropState();
    public static IntakeBaseState baseState = new IntakeBaseState();
    public static BHighPurple bHighPurple = new BHighPurple();
    public static BMidPurple bMidPurple = new BMidPurple();
    public static BHybrid bHybrid = new BHybrid();
    public static BHighYellow bHighYellow = new BHighYellow();
    public static BMidYellow bMidYellow = new BMidYellow();
    public static FHighPurple fHighPurple = new FHighPurple();
    public static FMidPurple fMidPurple = new FMidPurple();
    public static FHybrid fHybrid = new FHybrid();
    public static FHighYellow fHighYellow = new FHighYellow();
    public static FMidYellow fMidYellow = new FMidYellow();

    public ArmStateMachine() {

        idleState.build();
        intakeState.build();
        outtakeState.build();
        dropState.build();

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

        setCurrentState(idleState);
    }
}
