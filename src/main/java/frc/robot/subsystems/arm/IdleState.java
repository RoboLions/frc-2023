// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.*;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.State;
import frc.robot.lib.Transition;

/** Add your docs here. */
public class IdleState extends State {

    private static XboxController manipulatorController = RobotMap.manipulatorController;
    
    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void exit() {
        
    }

    public List<Transition> transitions = new ArrayList<Transition>(
        Arrays.asList(
            new Transition(() -> {
                return manipulatorController.getAButton() && !RobotMap.arm.getClawSensor();
            }, ArmStateMachine.intakeState),
            new Transition(() -> {
                return manipulatorController.getBButton();
            }, ArmStateMachine.dropState))
    );
}
