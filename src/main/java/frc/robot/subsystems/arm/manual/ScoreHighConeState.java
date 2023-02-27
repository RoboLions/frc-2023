// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.manual;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class ScoreHighConeState extends State {

    @Override
    public void build() {
        // go to idle state
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.idleState));

        // go to mid cone score state
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MID_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CONE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreMidConeState));

        // go to low cone score state
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.LOW_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CONE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreLowConeState));

        // go to scoring state
        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawAxis(Constants.DriverControls.SCORING_BUTTON) > 0.25;
        }, ArmStateMachine.scoringState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.HIGH_SCORE_CONE.SHOULDER_POSITION, 
            Constants.HIGH_SCORE_CONE.ELBOW_POSITION
        );
    }

    @Override
    public void execute() {
    }

    @Override
    public void exit() {
        
    }

}
