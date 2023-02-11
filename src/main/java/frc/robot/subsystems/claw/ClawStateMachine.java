// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import frc.robot.lib.statemachine.StateMachine;

/** Add your docs here. */
public class ClawStateMachine extends StateMachine {
    
    public static OpenState openState = new OpenState();
    public static ClosedCone closedCone = new ClosedCone();
    public static ClosedCube closedCube = new ClosedCube();

    public ClawStateMachine() {
        openState.build();
        closedCone.build();
        closedCube.build();

        setCurrentState(openState);
    }
}
