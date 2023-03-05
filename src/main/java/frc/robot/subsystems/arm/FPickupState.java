// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
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

        // TODO: our current default state is the closed cube state
        // transitions.add(new Transition(() -> {
        //     return (RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCone || RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCube);
        // }, ArmStateMachine.elbowIdleState));
    }

    @Override
    public void init() {
        // TODO: request claw open somehow, can't be in init because we need to open the claw after the arm moves a bit
        // Claw.requestClawOpen();

        RobotMap.arm.moveArmPosition(
            1.0 * Constants.GROUND_INTAKE.SHOULDER_POSITION, 
            1.0 * Constants.GROUND_INTAKE.ELBOW_POSITION
        );
    }

    @Override
    public void execute() {

        // TODO: can't close claw as soon as arm arrives, need to depend on if we have piece
        // if (RobotMap.arm.getArrived(Constants.GROUND_INTAKE.ALLOWANCE, Constants.GROUND_INTAKE.TIME)) {
        //     Claw.requestClawClosed();
        // }

    }

    @Override
    public void exit() {

        // TODO: need to send power to close motor throughout pickup to idle states
    }

}
