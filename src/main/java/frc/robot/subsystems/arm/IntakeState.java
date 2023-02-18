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

    static final double FIRST_STAGE_POSITION = Constants.IntakeState.FIRST_STAGE_POSITION;
    static final double SECOND_STAGE_POSITION = Constants.IntakeState.SECOND_STAGE_POSITION;
    static final double WRIST_POSITION = Constants.IntakeState.WRIST_POSITION;

    @Override
    public void build() {
        // idle button == T or claw is closed
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton() || RobotMap.arm.getClawClosed();
        }, ArmStateMachine.idleState));
    }

    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(FIRST_STAGE_POSITION, SECOND_STAGE_POSITION, WRIST_POSITION);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
        
    }

}