// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class PickupState extends State {
    
    private static XboxController manipulatorController = RobotMap.manipulatorController;

    static final double FIRST_STAGE_POSITION = Constants.PickupState.FIRST_STAGE_POSITION;
    static final double SECOND_STAGE_POSITION = Constants.PickupState.SECOND_STAGE_POSITION;
    static final double WRIST_POSITION = Constants.PickupState.WRIST_POSITION;

    @Override 
    public void build() {
        // idle if idle button or claw sensor == T
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton() || 
            (RobotMap.claw.getColor() != RobotMap.coneColor || 
            RobotMap.claw.getColor() != RobotMap.cubeColor);
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
