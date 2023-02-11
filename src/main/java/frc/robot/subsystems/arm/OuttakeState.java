// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class OuttakeState extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    double firstStagePosition = 0.0;
    double secondStagePosition = 0.0;
    double wristPosition = 0.0;
    double allowance = 1000.0;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.arm.getArrived(allowance) && RobotMap.arm.getClawOpen();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {

    }

    @Override
    public void execute() {
        RobotMap.arm.moveArmPosition(firstStagePosition, secondStagePosition, wristPosition);

        // if arm has arrived at position, send open request to claw
        if (RobotMap.arm.getArrived(allowance)) {
            RobotMap.openRequest = true;
        }
    }

    @Override
    public void exit() {
        
    }

}