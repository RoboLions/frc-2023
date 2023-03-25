// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

import com.ctre.phoenix.motorcontrol.ControlMode;

/** Add your docs here. */
public class IdleState extends State {
    
    @Override
    public void build() {

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.INTAKE_AXIS) > 0.25;
        }, IntakeStateMachine.intakingState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.OUTTAKE_BUTTON);
        }, IntakeStateMachine.outtakingState));

        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawAxis(Constants.DriverControls.SCORING_AXIS) > 0.25;
        }, IntakeStateMachine.outtakingState));
        
    }

    @Override
    public void init() {
        RobotMap.intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
    }
}
