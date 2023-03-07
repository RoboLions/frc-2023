// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import frc.robot.lib.statemachine.StateMachine;

/** Add your docs here. */
public class ClawStateMachine extends StateMachine {
    
    public static OpenState openState = new OpenState();
    public static OpeningState openingState = new OpeningState();
    public static ClosedState closedState = new ClosedState();
    public static ClosingState closingState = new ClosingState();

    public ClawStateMachine() {
        openState.build();
        openingState.build();
        closedState.build();
        closingState.build();

        setCurrentState(closedState);
    }
}
