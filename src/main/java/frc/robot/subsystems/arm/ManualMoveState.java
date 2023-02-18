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
        // deadband
        if ((wristInput > 0 && wristInput < 0.25) || (wristInput < 0 && wristInput > -0.25)) {
            wristInput = 0.0;
        }
        RobotMap.wristMotor.set(ControlMode.PercentOutput, wristInput);

        double elbowInput = manipulatorController.getLeftY();
        // deadband
        if ((elbowInput > 0 && elbowInput < 0.25) || (elbowInput < 0 && elbowInput > -0.25)) {
            elbowInput = 0.0;
        }
        RobotMap.elbowMotor.set(ControlMode.PercentOutput, elbowInput);

        double shoulderInput = manipulatorController.getRightY();
        // deadband
        if ((shoulderInput > 0 && shoulderInput < 0.25) || (shoulderInput < 0 && shoulderInput > -0.25)) {
            shoulderInput = 0.0;
        }
        RobotMap.shoulderMotor.set(ControlMode.PercentOutput, shoulderInput);
    }

    @Override
    public void exit() {
        
    }
}
