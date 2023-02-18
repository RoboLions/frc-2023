// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.front;

import frc.robot.lib.statemachine.State;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class FHighPurple extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    @Override
    public void build() {
        // return to idle automatically after scored
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));

        // transition to mid level
        transitions.add(new Transition(() -> {
            return manipulatorController.getYButton();
        }, ArmStateMachine.fMidPurple));

        // transition to hybrid level
        transitions.add(new Transition(() -> {
            return manipulatorController.getAButton();
        }, ArmStateMachine.fHybrid));

        // return to idle manually
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.FHighPurple.firstStagePosition, 
            Constants.FHighPurple.secondStagePosition, 
            Constants.FHighPurple.wristPosition
        );
    }

    @Override
    public void execute() {
        /* if arm has arrived at position and stayed at position for x seconds, 
        send open request to claw */
        if (RobotMap.arm.getArrived(Constants.FHighPurple.allowance, Constants.FHighPurple.time)) {
            RobotMap.openRequest = true;
        }
    }

    @Override
    public void exit() {
        
    }

}
