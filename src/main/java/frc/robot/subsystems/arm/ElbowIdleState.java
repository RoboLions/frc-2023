// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Arm;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ElbowIdleState extends State {

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.leftElbowMotor.getSelectedSensorPosition() < Constants.ELBOW_IDLE.ELBOW_POSITION;
        }, ArmStateMachine.idleState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MANUAL_MODE_BUTTON);
        }, ArmStateMachine.manualMoveState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.SUBSTATION_INTAKE_BUTTON);
        }, ArmStateMachine.substationIntakeState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.GROUND_INTAKE_FRONT) > 0.25;
        }, ArmStateMachine.groundPickupState));
    }
    
    @Override
    public void init(State prevState) {
        RobotMap.arm.setElbowIdle();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit(State nextState) {
        
    }
}
    
