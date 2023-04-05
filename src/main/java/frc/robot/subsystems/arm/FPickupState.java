// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.LED.LEDStateMachine;

/** Add your docs here. */
public class FPickupState extends State {
   
    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.elbowIdleState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MANUAL_MODE_BUTTON);
        }, ArmStateMachine.manualMoveState));
        
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.SUBSTATION_INTAKE_BUTTON);
        }, ArmStateMachine.substationIntakeState));
    }

    @Override
    public void init(State prevState) {
        if (RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState) {
            RobotMap.arm.moveArmPosition(
                Constants.GROUND_INTAKE_CONE.SHOULDER_POSITION, 
                Constants.GROUND_INTAKE_CONE.ELBOW_POSITION
            );
        } else {
            RobotMap.arm.moveArmPosition(
                Constants.GROUND_INTAKE_CUBE.SHOULDER_POSITION, 
                Constants.GROUND_INTAKE_CUBE.ELBOW_POSITION
            );
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void exit(State nextState) {

    }
}