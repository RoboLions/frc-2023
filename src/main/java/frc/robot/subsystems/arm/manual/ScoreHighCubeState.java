// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.manual;

import frc.robot.lib.statemachine.State;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class ScoreHighCubeState extends State {

    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.idleState));

        /*transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MID_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CUBE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreMidCubeState));

        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.LOW_SCORE_BUTTON) && 
            RobotMap.manipulatorController.getRawAxis(Constants.ManipulatorControls.MANUAL_CUBE_INDICATOR) > 0.25;
        }, ArmStateMachine.scoreLowCubeState));

        // Go to scoring Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawAxis(Constants.DriverButtons.SCORING_BUTTON) > 0.25;
        }, ArmStateMachine.scoringState));*/
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.HIGH_SCORE_CUBE.SHOULDER_POSITION, 
            Constants.HIGH_SCORE_CUBE.ELBOW_POSITION
        );
    }

    @Override
    public void execute() {
    }

    @Override
    public void exit() {
        
    }

}
