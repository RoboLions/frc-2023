// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ScoreState extends State {

    private static XboxController driverController = RobotMap.driverController;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return driverController.getBButton(); // transition to regular swerve
        }, DrivetrainStateMachine.teleopState));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
    /* 
    dpad right:
        if(driverController.getPOV() = 90){
        get rising edge (how many times individually pressed)
        move pos (# of rising edges) to right grids until reached right end
    }
    dpad left:
        if(driverController.getPOV() = 270){
        get rising edge (how many times individually pressed)
        move pos (# of rising edges) to left grids until reached left end
    }

    */
    }
    
}
