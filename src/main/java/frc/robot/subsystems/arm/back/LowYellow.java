// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm.back;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;

/** Add your docs here. */
public class LowYellow extends State {
    
    double firstStagePosition = 0.0;
    double secondStagePosition = 0.0;
    double wristPosition = 0.0;
    
    @Override
    public void init() {

    }

    @Override
    public void execute() {
        RobotMap.arm.moveArmPosition(firstStagePosition, secondStagePosition, wristPosition);
    }

    @Override
    public void exit() {
        
    }

}
