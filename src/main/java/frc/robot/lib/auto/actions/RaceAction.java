// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.auto.actions;

/**
 * Composite action, running all sub-actions at the same time All actions are
 * started then updated until all actions
 * report being done.
 *
 */
public class RaceAction implements Action {

    private final Action[] mActions;

    private final Action leadAction;

    public RaceAction(Action leadAction, Action... actions) {
        mActions = actions;
        this.leadAction = leadAction;
    }

    @Override
    public boolean isFinished() {
        if (leadAction.isFinished()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void update() {
        leadAction.update();
        for (Action action : mActions) {
            action.update();
        }
    }

    @Override
    public void done() {
        leadAction.done();
        for (Action action : mActions) {
            action.done();
        }
    }

    @Override
    public void start() {
        leadAction.start();
        for (Action action : mActions) {
            action.start();
        }
    }
}