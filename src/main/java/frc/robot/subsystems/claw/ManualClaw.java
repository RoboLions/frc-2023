// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ManualClaw extends State {
    
    double clawInput = 0.0;

    @Override
    public void build() {
        
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {
        
        // TODO: figure out how to manually control claw
        //double clawInput = RobotMap.manipulatorController.getRawAxis(Constants.DriverButtons.MA);

        if (RobotMap.driverController.getRawAxis(Constants.DriverButtons.MANUAL_OPEN_CLAW) > 0.25) {
            clawInput = 0.15;
        } else if (RobotMap.driverController.getRawAxis(Constants.DriverButtons.MANUAL_CLOSE_CLAW) > 0.25) {
            clawInput = -0.15;
        } else {
            clawInput = 0.0;
        }

        RobotMap.clawMotor.set(ControlMode.PercentOutput, clawInput);
    }

    @Override
    public void exit() {
    }
    
}