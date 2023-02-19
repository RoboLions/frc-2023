// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.front;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class FPickupState extends State {
   
    @Override
    public void build() {
        // idle if idle button or claw sensor == T
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorButtons.IDLE_BUTTON) || 
            (RobotMap.claw.getColor() == Constants.Claw.CONE_COLOR || 
            RobotMap.claw.getColor() == Constants.Claw.CUBE_COLOR);
        }, ArmStateMachine.idleState));
    }

    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.FPickupState.SHOULDER_POSITION, 
            Constants.FPickupState.ELBOW_POSITION, 
            Constants.FPickupState.WRIST_POSITION
        );
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
        
    }

}
