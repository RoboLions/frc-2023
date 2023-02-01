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
public class ArmStateMachine extends StateMachine {

    public IdleState idleState = new IdleState();
    public IntakeState intakeState = new IntakeState();

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    public ArmStateMachine() {

        Supplier<Boolean> checkIdleButton = () -> {
            // TODO: what button should be idle button
            return manipulatorController.getAButton();
        };

        Supplier<Boolean> checkIntakeButton = () -> {
            // TODO: what button should be intake button
            return manipulatorController.getBButton() && getClawSensor();
        };

        Supplier<Boolean> checkClawSensor = () -> {
            // TODO: make method
            return getClawSensor();
        };

        intakeState.addTransition(new Transition(checkIdleButton, idleState));
        intakeState.addTransition(new Transition(checkClawSensor, idleState));
        idleState.addTransition(new Transition(checkIntakeButton, intakeState));

        setCurrentState(idleState);
    }
}
