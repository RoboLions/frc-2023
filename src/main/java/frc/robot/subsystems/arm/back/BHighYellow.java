// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.back;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Add your docs here. */
public class BHighYellow extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return manipulatorController.getYButton() &&
            manipulatorController.getLeftTriggerAxis() > .25 &&
            RobotMap.arm.getColorSensor() == "yellow"; 
        }, ArmStateMachine.openState));
        transitions.add(new Transition(() -> {
            return manipulatorController.getBButton();
        }, ArmStateMachine.idleState));

        // open claw manually
        transitions.add(new Transition(() -> {
            return manipulatorController.getLeftBumper();
        }, ClawStateMachine.openState));
    }
    
    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        
    }

}
