// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class OuttakeState extends State {

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getArrived(Constants.OuttakeState.ALLOWANCE, Constants.OuttakeState.TIME) && RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.OuttakeState.SHOULDER_POSITION, 
            Constants.OuttakeState.ELBOW_POSITION, 
            Constants.OuttakeState.WRIST_POSITION);
    }

    @Override
    public void execute() {
    }

    @Override
    public void exit() {
        
    }

}