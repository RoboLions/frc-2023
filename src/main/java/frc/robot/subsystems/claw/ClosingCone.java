// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ClosingCone extends State {
    
    private Timer timer = new Timer();
    
    @Override
    public void build() {
        // TODO: change time
        // claw is now closed on a cone after 0.5 seconds
        transitions.add(new Transition(() -> {
            return timer.hasElapsed(0.5);
        }, ClawStateMachine.closedCone));
    }

    @Override
    public void init() {
        RobotMap.claw.setClawClosedCone();
        timer.start();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void exit() {
        timer.stop();
        timer.reset();
    }
}
