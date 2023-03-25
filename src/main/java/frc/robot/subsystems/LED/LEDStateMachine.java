// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.StateMachine;

/** Add your docs here. */
public class LEDStateMachine extends StateMachine {

    public static CubeLEDState cubeLEDState = new CubeLEDState();
    public static ConeLEDState coneLEDState = new ConeLEDState();

    public LEDStateMachine() {

        cubeLEDState.build();
        coneLEDState.build();

        setCurrentState(coneLEDState);
    }
}