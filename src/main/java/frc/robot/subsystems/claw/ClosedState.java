// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ClosedState extends State {
    
    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return Claw.openRequest;
        }, ClawStateMachine.openingState));

        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawAxis(Constants.DriverControls.CLOSE_BUTTON) > 0.25;
        }, ClawStateMachine.closingState));
    }

    @Override
    public void init() {
        RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
    }
}
