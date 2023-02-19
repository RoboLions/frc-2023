// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.statemachine.StateMachine;
import frc.robot.subsystems.arm.back.*;
import frc.robot.subsystems.arm.front.*;

public class ArmStateMachine extends StateMachine {

    public static IdleState idleState = new IdleState();
    public static FIntakeState fIntakeState = new FIntakeState();
    public static BIntakeState bIntakeState = new BIntakeState();
    public static ManualMoveState manualMoveState = new ManualMoveState();
    public static OuttakeState outtakeState = new OuttakeState();
    public static FPickupState fPickupState = new FPickupState();
    public static BPickupState bPickupState = new BPickupState();
    public static ScoreHighState scoreHighState = new ScoreHighState();
    public static ScoreMidState scoreMidState = new ScoreMidState();
    public static ScoreLowState scoreLowState = new ScoreLowState();
    public static ScoringState scoringState = new ScoringState();

    public ArmStateMachine() {

        idleState.build();
        fIntakeState.build();
        bIntakeState.build();
        manualMoveState.build();
        outtakeState.build();
        fPickupState.build();
        bPickupState.build();
        scoreHighState.build();
        scoreMidState.build();
        scoreLowState.build();
        scoringState.build();

        setCurrentState(idleState);
    }
}
