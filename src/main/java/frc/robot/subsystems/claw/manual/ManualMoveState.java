// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw.manual;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Add your docs here. */
public class ManualMoveState extends State {
    
    double clawInput = 0.0;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.SWITCH_TO_NORMAL_CLAW_BUTTON);
        }, ClawStateMachine.openingState));

        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.MANUAL_CLOSE_CUBE_BUTTON);
        }, ClawStateMachine.manualCubeState));

        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.MANUAL_CLOSE_CONE_BUTTON);
        }, ClawStateMachine.manualConeState));

        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.MANUAL_OPEN_CLAW_BUTTON);
        }, ClawStateMachine.manualOpenState));
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {

        if (RobotMap.driverController.getRawAxis(Constants.DriverControls.MANUAL_TRIGGER_OPEN_CLAW) > 0.25) {
            clawInput = 0.8;
        } else if (RobotMap.driverController.getRawAxis(Constants.DriverControls.MANUAL_TRIGGER_CLOSE_CLAW) > 0.25) {
            clawInput = -0.8;
        } else {
            clawInput = 0.0;
        }

        RobotMap.clawMotor.set(ControlMode.PercentOutput, clawInput);
    }

    @Override
    public void exit() {
    }
}
