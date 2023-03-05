// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;

/** Add your docs here. */
public class ManualClaw extends State {
    
    double clawInput = 0.0;

    public static double setpoint = 0.0;
    double feedback = 0.0;
    double command = 0.0;

    @Override
    public void build() {
        
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {
        
        feedback = Math.abs(RobotMap.clawEncoder.get());
        setpoint = 50.0;

        // if (RobotMap.driverController.getLeftTriggerAxis() > 0.25) {
        //     setpoint = 0.0;
        //     command = Claw.clawPID.execute(setpoint, feedback);
        //     RobotMap.clawMotor.set(ControlMode.PercentOutput, command);
        // }
        
        // if (RobotMap.driverController.getRightTriggerAxis() > 0.25) {
        //     setpoint = 50.0;
        //     command = -Claw.clawPID.execute(setpoint, feedback);
        //     RobotMap.clawMotor.set(ControlMode.PercentOutput, command);
        // }
        
        // if (command != 0.0) {
        //     System.out.println("Command: " + command + ", feedback: " + feedback);
        // }

        System.out.println("Feedback: " + feedback + ", setpoint: " + setpoint);

        if (feedback < setpoint) {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, -0.6);
        } else {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    @Override
    public void exit() {
    }
    
}
