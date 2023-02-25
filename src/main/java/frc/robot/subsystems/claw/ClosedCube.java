// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ClosedCube extends State {
    
    @Override
    public void build() {
        // if we don't detect a cube, open the claw
        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() != Constants.CLAW.CUBE_COLOR && !Claw.closeRequest;
        }, ClawStateMachine.openingState));

        transitions.add(new Transition(() -> {
            return Claw.openRequest;
        }, ClawStateMachine.openingState));

        // open the claw if driver presses right trigger
        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawAxis(Constants.DriverButtons.SCORING_BUTTON) > Constants.STICK_DEADBAND;
        }, ClawStateMachine.openingState));
    }

    @Override
    public void init() {
        RobotMap.claw.setClawClosedCube();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
        // set openRequest to false
        Claw.openRequest = false;
    }
}
