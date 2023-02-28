// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.statemachine;

import java.util.List;

/** Class for creating a subsystem's state machine */
public class StateMachine {

    State currentState;

    // returns the current state of the state machine
    public State getCurrentState() {
        return currentState;
    }

    /* executes the current state, 
       then checks if any transitions are true to transition to the next state */
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

    /* checks the current state isn't null before exiting the state and 
    setting the new state as the current state */
    public void setCurrentState(State newState) {
        if (currentState != null) {
            currentState.exit_private();
        }
        currentState = newState;
        currentState.state_machine_name = this.getClass().getName();
        currentState.init_private();
    }
}
