// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class OuttakingState extends State {
    
    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, IntakeStateMachine.idleState));
    }

    @Override
    public void init() {
        RobotMap.intake.runOuttake();
    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        
    }
}
