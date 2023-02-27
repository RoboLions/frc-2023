// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import frc.robot.lib.statemachine.StateMachine;
import frc.robot.subsystems.claw.manual.ManualConeState;
import frc.robot.subsystems.claw.manual.ManualCubeState;
import frc.robot.subsystems.claw.manual.ManualMoveState;
import frc.robot.subsystems.claw.manual.ManualOpenState;

/** Add your docs here. */
public class ClawStateMachine extends StateMachine {
    
    public static OpenState openState = new OpenState();
    public static OpeningState openingState = new OpeningState();
    public static ClosedCone closedCone = new ClosedCone();
    public static ClosedCube closedCube = new ClosedCube();
    public static ClosingCone closingCone = new ClosingCone();
    public static ClosingCube closingCube = new ClosingCube();

    /* manual states */
    public static ManualMoveState manualMoveState = new ManualMoveState();
    public static ManualConeState manualConeState = new ManualConeState();
    public static ManualCubeState manualCubeState = new ManualCubeState();
    public static ManualOpenState manualOpenState = new ManualOpenState();

    public ClawStateMachine() {
        openState.build();
        openingState.build();
        closedCone.build();
        closedCube.build();
        closingCone.build();
        closingCube.build();
        manualMoveState.build();
        manualConeState.build();
        manualCubeState.build();
        manualOpenState.build();

        setCurrentState(openState);
        // setCurrentState(manualClaw);
    }
}
