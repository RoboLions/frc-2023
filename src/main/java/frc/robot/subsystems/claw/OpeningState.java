// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class OpeningState extends State {
    
    private Timer openingStateTimer = new Timer();
    
    @Override
    public void build() {
        // claw is now open after x seconds
        transitions.add(new Transition(() -> {
            return openingStateTimer.get() > (Constants.CLAW.TIME_OPEN_CLAW + 0.05);
        }, ClawStateMachine.openState));
    }

    @Override
    public void init() {
        openingStateTimer.start();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Claw opening timer", openingStateTimer.get());

        // apply power to claw motor for 1 second to open the claw
        if (openingStateTimer.hasElapsed(Constants.CLAW.TIME_OPEN_CLAW)) {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.0);
        } else {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, Constants.CLAW.OPEN_POWER);
        }
    }

    @Override
    public void exit() {
        openingStateTimer.stop();
        openingStateTimer.reset();
    }
}
