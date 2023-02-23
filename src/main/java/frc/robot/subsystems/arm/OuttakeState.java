// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class OuttakeState extends State {

    private static boolean openRequested = false;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButtonPressed(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.idleState));

        transitions.add(new Transition(() -> {
            return RobotMap.claw.isOpen();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {
        openRequested = false;
        RobotMap.arm.moveArmPosition(
            Constants.OUTTAKE_STATE.SHOULDER_POSITION, 
            Constants.OUTTAKE_STATE.ELBOW_POSITION
        );
    }

    @Override
    public void execute() {
        if (RobotMap.arm.getArrived(Constants.OUTTAKE_STATE.ALLOWANCE, Constants.OUTTAKE_STATE.TIME) && !openRequested) {
            Claw.openRequest = true;
        }
    }

    @Override
    public void exit() {
        
    }

}