// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.front;

import frc.robot.lib.statemachine.State;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class FHighYellow extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    double firstStage = 0.0;
    double secondStage = 0.0;
    double wrist = 0.0;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return manipulatorController.getYButton(); 
        }, ClawStateMachine.openState));
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton();
        }, ArmStateMachine.idleState));
    }
    
    @Override
    public void init() {

    }

    @Override
    public void execute() {
        RobotMap.arm.moveArmPosition(firstStage, secondStage, wrist);
    }

    @Override
    public void exit() {
        
    }

}
