// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.ManipulatorControls;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ManualMoveState extends State {

    @Override
    public void build() {
        // idle if idle button
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.idleState));
    }

    @Override
    public void init() {
    }

    @Override
    public void execute() {

        if (RobotMap.driverController.getRawAxis(Constants.DriverControls.CLOSE_BUTTON) > 0.25) {
            Claw.requestClawClosed();
        }
        
        double elbowInput = RobotMap.manipulatorController.getRawAxis(ManipulatorControls.ELBOW_AXIS);
        RobotMap.leftElbowMotor.set(ControlMode.PercentOutput, RobotMap.arm.applyDeadband(elbowInput));

        double shoulderInput = RobotMap.manipulatorController.getRawAxis(ManipulatorControls.SHOULDER_AXIS);
        RobotMap.leftShoulderMotor.set(ControlMode.PercentOutput, RobotMap.arm.applyDeadband(-shoulderInput));
    }

    @Override
    public void exit() {
        RobotMap.arm.resetEncoders();
    }
}