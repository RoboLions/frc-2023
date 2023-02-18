// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ManualMoveState extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    @Override
    public void build() {
        // idle if idle button
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton();
        }, ArmStateMachine.idleState));
    }

    @Override
    public void init() {
    }

    @Override
    public void execute() {
        // TODO: figure out left or right being open or close
        double wristInput = manipulatorController.getLeftTriggerAxis() - manipulatorController.getRightTriggerAxis();
        RobotMap.wristMotor.set(ControlMode.PercentOutput, RobotMap.arm.applyDeadband(wristInput));

        double elbowInput = manipulatorController.getLeftY();
        RobotMap.elbowMotor.set(ControlMode.PercentOutput, RobotMap.arm.applyDeadband(elbowInput));

        double shoulderInput = manipulatorController.getRightY();
        RobotMap.shoulderMotor.set(ControlMode.PercentOutput, RobotMap.arm.applyDeadband(shoulderInput));
    }

    @Override
    public void exit() {
        
    }
}
