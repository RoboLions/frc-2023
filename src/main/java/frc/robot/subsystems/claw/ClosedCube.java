// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ClosedCube extends State {
    
    @Override
    public void build() {
        // TODO: make open request method
        // open claw if open request
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getOpenRequest();
        }, ClawStateMachine.openState));
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {
        RobotMap.clawLib.setClawClosedCube();
    }

    @Override
    public void exit() {
        
    }
}
