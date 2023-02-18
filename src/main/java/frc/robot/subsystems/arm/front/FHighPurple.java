// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.front;

import frc.robot.lib.statemachine.State;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class FHighPurple extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    static final double FIRST_STAGE_POSITION = Constants.FHighPurple.FIRST_STAGE_POSITION;
    static final double SECOND_STAGE_POSITION = Constants.FHighPurple.SECOND_STAGE_POSITION;
    static final double WRIST_POSITION = Constants.FHighPurple.WRIST_POSITION;
    static final double ALLOWANCE = Constants.FHighPurple.ALLOWANCE;
    static final double TIME = Constants.FHighPurple.TIME;

    @Override
    public void build() {
        // return to idle automatically after scored
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));

        // return to idle manually
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton();
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
