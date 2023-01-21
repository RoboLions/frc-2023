// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.function.Supplier;

/** Class for adding a transition to a state */
public class Transition {

    private Supplier<Boolean> transition_function;

    private State state;

    // constructor to initialize a new transition object
    public Transition(Supplier<Boolean> func, State next_state) {
        transition_function = func;
        state = next_state;
    }

    // check if the transition function returns true
    public boolean check() {
        return transition_function.get();
    }

    // get the current state of the state machine
    public State getState() {
        return state;
    }
}
