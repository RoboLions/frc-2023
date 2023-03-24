// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.lib.statemachine.StateMachine;

/** Add your docs here. */
public class IntakeStateMachine extends StateMachine {
    
    public static IdleState idleState = new IdleState();
    public static IntakingState intakingState = new IntakingState();
    public static OuttakingState outtakingState = new OuttakingState();

    public IntakeStateMachine() {
        idleState.build();
        intakingState.build();
        outtakingState.build();

        setCurrentState(idleState);
    }
}
