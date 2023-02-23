// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class GroundPickupState extends State {
   
    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.idleState));

        transitions.add(new Transition(() -> {
            return RobotMap.claw.getColor() != null &&
                RobotMap.claw.isClosed();
        }, ArmStateMachine.idleState));
    }

    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.GROUND_INTAKE.LEFT_SHOULDER_POSITION, 
            Constants.GROUND_INTAKE.RIGHT_SHOULDER_POSITION, 
            Constants.GROUND_INTAKE.LEFT_ELBOW_POSITION,
            Constants.GROUND_INTAKE.RIGHT_ELBOW_POSITION
        );
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
        
    }

}
