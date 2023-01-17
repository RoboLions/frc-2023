// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.List;

/** Add your docs here. */
public class StateMachine {

    State currentState;

    public State getCurrentState() {
        return currentState;
    }

    public State getNextState() {
        List<Transition> transitions = currentState.getTransitions();

        for (int i = 0; i < transitions.size(); i++) {
            Transition t = transitions.get(i);
            if (t.check()) {
                return t.getState();
            }
        }

        return currentState;
    }

    public void setCurrentState(State newState) {
        currentState = newState;
    }
}
