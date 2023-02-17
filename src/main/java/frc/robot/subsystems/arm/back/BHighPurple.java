// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.back;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class BHighPurple extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    @Override
    public void build() {
        // return to idle automatically after scored
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));

        // return to idle manually
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.BHighPurple.firstStagePosition, 
            Constants.BHighPurple.secondStagePosition, 
            Constants.BHighPurple.wristPosition
        );
    }

    @Override
    public void execute() {
        /* if arm has arrived at position and stayed at position for x seconds, 
        send open request to claw */
        if (RobotMap.arm.getArrived(Constants.BHighPurple.allowance, Constants.BHighPurple.time)) {
            RobotMap.openRequest = true;
        }
    }

    @Override
    public void exit() {
        
    }

}
