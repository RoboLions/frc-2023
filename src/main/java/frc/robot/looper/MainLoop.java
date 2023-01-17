// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.looper;

import frc.robot.lib.StateMachine;
import frc.robot.lib.State;
import frc.robot.lib.Transition;

import java.util.function.Function;

import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class MainLoop {

    StateMachine machine;
    State AButton;
    State BButton;
    State XButton;

    public XboxController manipulatorController = new XboxController(1);

    MainLoop() {
        AButton = new State((x -> {
                    System.out.println("I am entering state A");
                }), );
        AButton.addTransition(new Transition((x -> {
            // is button pressed
        }), BButton));
    }

}
