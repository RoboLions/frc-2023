// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.StateMachine;

/** Add your docs here. */
public class ArmStateMachine extends StateMachine {

    public static IdleState idleState = new IdleState();
    public static IntakeState intakeState = new IntakeState();
    public static OuttakeState outtakeState = new OuttakeState();
    public static DropState dropState = new DropState();

    public ArmStateMachine() {

        idleState.build();
        intakeState.build();
        outtakeState.build();
        dropState.build();

        setCurrentState(idleState);
    }
}
