// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.statemachine.State;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Add your docs here. */
public class SubstationIntakeState extends State {

    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.elbowIdleState));

        transitions.add(new Transition(() -> {
            return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCone || RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCube;
        }, ArmStateMachine.elbowIdleState));
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
    }

    @Override
    public void exit() {
        
    }

}
