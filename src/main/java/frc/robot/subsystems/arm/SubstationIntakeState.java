// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.statemachine.State;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class SubstationIntakeState extends State {
    
    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.elbowIdleState));

       transitions.add(new Transition(() -> {
        return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MANUAL_MODE_BUTTON);
       }, ArmStateMachine.manualMoveState));
       
        // pickup from ground
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.GROUND_INTAKE_FRONT) > 0.25;
        }, ArmStateMachine.groundPickupState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.SUBSTATION_INTAKE.SHOULDER_POSITION, 
            Constants.SUBSTATION_INTAKE.ELBOW_POSITION
        );
    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
     
    }
}
