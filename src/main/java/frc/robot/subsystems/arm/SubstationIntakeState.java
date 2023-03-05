// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.statemachine.State;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class SubstationIntakeState extends State {

    private int count = 0;
    
    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.elbowIdleState));

        // TODO: transition back to idle when we have a piece
        // transitions.add(new Transition(() -> {
        //     return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCone || RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCube;
        // }, ArmStateMachine.elbowIdleState));

        // if we hold a cone or cube
        // transitions.add(new Transition(() -> {
        //     return RobotMap.claw.getColor() == Constants.CLAW.CONE_COLOR || RobotMap.claw.getColor() == Constants.CLAW.CUBE_COLOR;
        // }, ArmStateMachine.elbowIdleState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.SUBSTATION_INTAKE.SHOULDER_POSITION, 
            Constants.SUBSTATION_INTAKE.ELBOW_POSITION
        );
    }

    @Override
    public void execute() {

        if (count < 10) {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, -0.8);
            count++;
        } else {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.0);
        }
    
    }

    @Override
    public void exit() {
        RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.8);
        for (int i = 0; i < 300; i++) {
            System.out.println(i);
        }
    }
}
