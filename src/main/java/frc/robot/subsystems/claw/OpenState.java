// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;

/** Add your docs here. */
public class OpenState extends State {
    
    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return !Claw.openRequest;
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
