// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.State;
import frc.robot.lib.Transition;

/** Add your docs here. */
public class IntakeState extends State {
private static XboxController manipulatorController = RobotMap.manipulatorController;
    public void build() {
        transitions.add(new Transition(() -> {
            return manipulatorController.getLeftBumper(); // || clawClosed == True;
        }, ArmStateMachine.idleState));
    }
    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        
    }

}