// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.statemachine.State;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.LED.LEDStateMachine;

/** Add your docs here. */
public class ScoreHighState extends State {

    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.elbowIdleState));

        // transition to mid level
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MID_SCORE_BUTTON);
        }, ArmStateMachine.scoreMidState));

        // transition to hybrid level
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.LOW_SCORE_BUTTON);
        }, ArmStateMachine.scoreLowState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MANUAL_MODE_BUTTON);
        }, ArmStateMachine.manualMoveState));
    }
    
    @Override
    public void init() {
        if (RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState) {
            RobotMap.arm.moveArmPosition(
                Constants.HIGH_SCORE_CONE.SHOULDER_POSITION, 
                Constants.HIGH_SCORE_CONE.ELBOW_POSITION
            );
        } else {
            RobotMap.arm.moveArmPosition(
                Constants.HIGH_SCORE_CUBE.SHOULDER_POSITION, 
                Constants.HIGH_SCORE_CUBE.ELBOW_POSITION
            );
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public void exit() {
        
    }

}
