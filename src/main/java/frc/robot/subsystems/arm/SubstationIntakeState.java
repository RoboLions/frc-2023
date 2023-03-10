// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.lib.interfaces.Arm;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Add your docs here. */
public class SubstationIntakeState extends State {

    private int count = 0;
    
    @Override
    public void build() {
        // Go to IDLE Transitions
        transitions.add(new Transition(() -> {
            return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.IDLE_BUTTON);
        }, ArmStateMachine.elbowIdleState));

       transitions.add(new Transition(() -> {
        return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedState;
       }, ArmStateMachine.elbowIdleState));

       transitions.add(new Transition(() -> {
        return RobotMap.manipulatorController.getRawButton(Constants.ManipulatorControls.MANUAL_MODE_BUTTON);
       }, ArmStateMachine.manualMoveState));
    }
    
    @Override
    public void init() {
        RobotMap.arm.moveArmPosition(
            Constants.SUBSTATION_INTAKE.SHOULDER_POSITION, 
            Constants.SUBSTATION_INTAKE.ELBOW_POSITION
        );
        Claw.requestClawOpen();
        count = 0; // reset the count each time
    }

    @Override
    public void execute() {
        // if (Arm.getArrived(Constants.SUBSTATION_INTAKE.ALLOWANCE, Constants.SUBSTATION_INTAKE.TIME) && Claw.getColor() != null && count < 1) {
        //     Claw.requestClawClosed();
        //     count++;
        // }
    
        if (RobotMap.driverController.getRawAxis(Constants.DriverControls.CLOSE_BUTTON) > 0.25) {
            Claw.requestClawClosed();
        }
    }

    @Override
    public void exit() {
     
    }
}
