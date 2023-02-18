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
public class FHybrid extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    @Override
    public void build() {
        // return to idle automatically after scored
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));

        // transition to mid level purple
        transitions.add(new Transition(() -> {
            return manipulatorController.getAButton() &&
            (RobotMap.claw.getColor() == RobotMap.cubeColor);
        }, ArmStateMachine.fMidPurple));

        // transition to high level purple
        transitions.add(new Transition(() -> {
            return manipulatorController.getYButton() && 
            (RobotMap.claw.getColor() == RobotMap.cubeColor);
        }, ArmStateMachine.fHighPurple));

        // transition to mid level yellow
        transitions.add(new Transition(() -> {
            return manipulatorController.getAButton() &&
            (RobotMap.claw.getColor() == RobotMap.coneColor);
        }, ArmStateMachine.fMidYellow));

        // transition to high level yellow
        transitions.add(new Transition(() -> {
            return manipulatorController.getYButton() && 
            (RobotMap.claw.getColor() == RobotMap.coneColor);
        }, ArmStateMachine.fHighYellow));

        // return to idle manually
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.FHybrid.firstStagePosition, 
            Constants.FHybrid.secondStagePosition, 
            Constants.FHybrid.wristPosition
        );
    }

    @Override
    public void execute() {
        /* if arm has arrived at position and stayed at position for x seconds, 
        send open request to claw */
        if (RobotMap.arm.getArrived(Constants.FHybrid.allowance, Constants.FHybrid.time)) {
            RobotMap.openRequest = true;
        }
    }
    
    @Override
    public void exit() {
        
    }

}
