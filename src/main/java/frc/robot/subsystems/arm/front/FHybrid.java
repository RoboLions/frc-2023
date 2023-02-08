// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.front;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.State;
import frc.robot.lib.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Add your docs here. */
public class FHybrid extends State {
    private static XboxController manipulatorController = RobotMap.manipulatorController;

    public void build(){
        transitions.add(new Transition(() -> {
            return manipulatorController.getLeftBumper() || 
            (!RobotMap.arm.getClawSensor() && manipulatorController.getRightTriggerAxis() < 0.25);
        }, ArmStateMachine.idleState));
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
