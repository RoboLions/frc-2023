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

    public void setNextState() {
        currentState.execute_private();
        List<Transition> transitions = currentState.getTransitions();

        for (int i = 0; i < transitions.size(); i++) {
            Transition t = transitions.get(i);
            if (t.check()) {
                setCurrentState(t.getState());
            }
        }
    }

    public void setCurrentState(State newState) {
        if (currentState != null) {
            currentState.exit_private();
        }
        currentState = newState;
        currentState.state_machine_name = this.getClass().getName();
        currentState.init_private();
    }
}
