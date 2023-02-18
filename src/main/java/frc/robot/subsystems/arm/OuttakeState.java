// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class OuttakeState extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    static final double FIRST_STAGE_POSITION = Constants.OuttakeState.FIRST_STAGE_POSITION;
    static final double SECOND_STAGE_POSITION = Constants.OuttakeState.SECOND_STAGE_POSITION;
    static final double WRIST_POSITION = Constants.OuttakeState.WRIST_POSITION;
    static final double ALLOWANCE = Constants.OuttakeState.ALLOWANCE;
    static final double TIME = Constants.OuttakeState.TIME;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getArrived(allowance, time) && RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(FIRST_STAGE_POSITION, SECOND_STAGE_POSITION, WRIST_POSITION);
    }

    @Override
    public void execute() {
        /* if arm has arrived at position and stayed at position for 0.5 seconds, 
        send open request to claw */
        if (RobotMap.arm.getArrived(allowance, time)) {
            RobotMap.openRequest = true;
        }
    }

    @Override
    public void exit() {
        
    }

}