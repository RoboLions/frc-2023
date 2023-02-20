// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ScoringState extends State {

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.idleState));

        transitions.add(new Transition(() -> {
            return RobotMap.claw.isOpen();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {
        Claw.openRequest = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        
    }

}