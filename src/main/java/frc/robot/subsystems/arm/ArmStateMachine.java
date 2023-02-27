// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.statemachine.StateMachine;
import frc.robot.subsystems.arm.manual.ManualMoveState;
import frc.robot.subsystems.arm.manual.ScoreHighConeState;
import frc.robot.subsystems.arm.manual.ScoreHighCubeState;
import frc.robot.subsystems.arm.manual.ScoreLowConeState;
import frc.robot.subsystems.arm.manual.ScoreLowCubeState;
import frc.robot.subsystems.arm.manual.ScoreMidConeState;
import frc.robot.subsystems.arm.manual.ScoreMidCubeState;

public class ArmStateMachine extends StateMachine {

    public static IdleState idleState = new IdleState();
    public static SubstationIntakeState substationIntakeState = new SubstationIntakeState();
    public static OuttakeState outtakeState = new OuttakeState();
    public static FPickupState groundPickupState = new FPickupState();
    public static ScoreHighState scoreHighState = new ScoreHighState();
    public static ScoreMidState scoreMidState = new ScoreMidState();
    public static ScoreLowState scoreLowState = new ScoreLowState();
    public static ScoringState scoringState = new ScoringState();

    /* manual states */
    public static ManualMoveState manualMoveState = new ManualMoveState();
    public static ScoreHighCubeState scoreHighCubeState = new ScoreHighCubeState();
    public static ScoreMidCubeState scoreMidCubeState = new ScoreMidCubeState();
    public static ScoreLowCubeState scoreLowCubeState = new ScoreLowCubeState();
    public static ScoreHighConeState scoreHighConeState = new ScoreHighConeState();
    public static ScoreMidConeState scoreMidConeState = new ScoreMidConeState();
    public static ScoreLowConeState scoreLowConeState = new ScoreLowConeState();

    public ArmStateMachine() {

        // deez.build();
        // nutz.build();
        
        idleState.build();
        substationIntakeState.build();
        outtakeState.build();
        groundPickupState.build();
        scoreHighState.build();
        scoreMidState.build();
        scoreLowState.build();
        scoringState.build();

        manualMoveState.build();
        scoreHighCubeState.build();
        scoreMidCubeState.build();
        scoreLowCubeState.build();
        scoreHighConeState.build();
        scoreMidConeState.build();
        scoreLowConeState.build();

        setCurrentState(idleState);
    }
}
