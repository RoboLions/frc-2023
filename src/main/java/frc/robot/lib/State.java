// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.*;
import java.util.function.Function;

/** Add your docs here. */
public class State {

    public List<Transition> transitions = new ArrayList<Transition>();
    State state;

    /*Function<String, Void> init_function;
    Function<String, Void> execute_function;
    Function<String, Void> exit_function;*/

    private Runnable init_function;
    private Runnable execute_function;
    private Runnable exit_function;

    public State() {}

    public State(List<Transition> _transitions) {
        transitions = _transitions;
    }

    /*public State(Function<String, Void> _init, Function<String, Void> _execute, Function<String, Void> _exit) {
        init_function = _init;
        execute_function = _execute;
        exit_function = _exit;
    }*/

    public State(Runnable _init, Runnable _execute, Runnable _exit) {
        init_function = _init;
        execute_function = _execute;
        exit_function = _exit;
    }

    public void addTransition(Transition transition) {
        transitions.add(transition);
    }

    public List<Transition> getTransitions() {
        return transitions;
    }

    public void init() {
        init_function.run();
    }

    public void execute() {
        execute_function.run();
    }

    public void exit() {
        exit_function.run();
    }
}
