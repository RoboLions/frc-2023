// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class OuttakeState extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getArrived(Constants.Outtake.allowance, Constants.Outtake.time) && RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.Outtake.shoulderPosition, 
            Constants.Outtake.elbowPosition, 
            Constants.Outtake.wristPosition);
    }

    @Override
    public void execute() {
        /* if arm has arrived at position and stayed at position for 0.5 seconds, 
        send open request to claw */
        if (RobotMap.arm.getArrived(Constants.Outtake.allowance, Constants.Outtake.time)) {
            RobotMap.openRequest = true;
        }
    }

    @Override
    public void exit() {
        
    }

}