// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.auto.actions;

import java.util.function.Supplier;

/** Add your docs here. */
public class ConditionAction implements Action {

    private Supplier<Boolean> mF;

    public ConditionAction(Supplier<Boolean> f) {
        this.mF = f;
    }

    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return this.mF.get();
    }

    @Override
    public void done() {}
}