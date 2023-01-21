// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/** Add your docs here. */
public class State {

    // Creating a class-based singleton
    private static State INSTANCE;

    public static State getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new State();
        }
        return INSTANCE;
    }

    protected State() {}

    public List<Transition> transitions = new ArrayList<Transition>();
    public String state_machine_name = "";

    public void addTransition(Transition transition) {
        transitions.add(transition);
    }

    public void addTransition(Supplier<Boolean> func, State next_state) {
        transitions.add(new Transition(func, next_state));
    }

    public List<Transition> getTransitions() {
        return transitions;
    }

    public void init() {}

    public void execute() {}

    public void exit() {}

    public void init_private() {
        System.out.println(state_machine_name + ": entering state " + this.getClass().getName());
        init();
    }

    public void execute_private() {
        execute();
    }

    public void exit_private() {
        System.out.println(state_machine_name + ": exiting state " + this.getClass().getName());
        exit();
    }
}
