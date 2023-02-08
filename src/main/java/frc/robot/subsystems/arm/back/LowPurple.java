// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.back;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.State;
import frc.robot.lib.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class LowPurple extends State {
    private static XboxController manipulatorController = RobotMap.manipulatorController;

    @Override
    public void init() {

    }

    public void build() {
        transitions.add(new Transition(() -> {
            return manipulatorController.getYButton();
        }, ArmStateMachine.idleState));
    }
    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        
    }

}
