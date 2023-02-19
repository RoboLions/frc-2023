// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.back;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class BIntakeState extends State {

    @Override
    public void build() {
        // idle button == T or claw is closed
        transitions.add(new Transition(() -> {
            return (RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.IDLE_BUTTON) 
                   || RobotMap.arm.getClawClosed());
        }, ArmStateMachine.idleState));
    }

    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.BIntakeState.SHOULDER_POSITION, 
            Constants.BIntakeState.ELBOW_POSITION, 
            Constants.BIntakeState.WRIST_POSITION);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
        
    }

}