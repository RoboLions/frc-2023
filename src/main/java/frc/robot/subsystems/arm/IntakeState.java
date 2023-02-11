// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class IntakeState extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    double firstStagePosition = 0.0;
    double secondStagePosition = 0.0;
    double wristPosition = 0.0;

    @Override
    public void build() {
        // idle button == T or claw is closed
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton() || RobotMap.arm.getClawClosed();
        }, ArmStateMachine.idleState));
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {
        RobotMap.arm.moveArmPosition(firstStagePosition, secondStagePosition, wristPosition);
    }

    @Override
    public void exit() {
        
    }

}