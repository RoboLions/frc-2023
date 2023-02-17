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
        RobotMap.armFirstStage.set(ControlMode.Position, Constants.ManualMove.firstStagePosition);
        RobotMap.armSecondStage.set(ControlMode.Position, Constants.ManualMove.secondStagePosition);
    }

    @Override
    public void execute() {
        double wristInput = manipulatorController.getLeftY();
        RobotMap.wrist.set(ControlMode.PercentOutput, wristInput);
    }

    @Override
    public void exit() {
        
    }
}
