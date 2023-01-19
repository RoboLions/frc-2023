// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.function.Function;
import java.util.function.Supplier;

/** Add your docs here. */
public class Transition {

    //Function<String, Boolean> transition_function;
    
    private Supplier<Boolean> transition_function;

    private State state;

    /*public Transition(Function<String, Boolean> func, State next_state) {
        transition_function = func;
        state = next_state;
    }*/

    public Transition(Supplier<Boolean> func, State next_state) {
        transition_function = func;
        state = next_state;
    }

    public boolean check() {
        return transition_function != null;
    }

    public State getState() {
        return state;
    }
}
