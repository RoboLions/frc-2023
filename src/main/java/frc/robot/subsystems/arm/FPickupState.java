// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Arm;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Add your docs here. */
public class FPickupState extends State {
   
    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.elbowIdleState));

        transitions.add(new Transition(() -> {
            return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedState;
        }, ArmStateMachine.elbowIdleState));
    }

    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.GROUND_INTAKE.SHOULDER_POSITION, 
            Constants.GROUND_INTAKE.ELBOW_POSITION
        );
        Claw.requestClawOpen();
    }

    @Override
    public void execute() {
        if (Arm.getArrived(Constants.GROUND_INTAKE.ALLOWANCE, Constants.GROUND_INTAKE.TIME) && Claw.getColor() != null) {
            Claw.requestClawClosed();
        }
    }

    @Override
    public void exit() {

        // TODO: need to send power to close motor throughout pickup to idle states
    }

}
