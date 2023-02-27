// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw.manual;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Add your docs here. */
public class ManualConeState extends State {

    private Timer manualConeTimer = new Timer();

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.MANUAL_OPEN_CLAW_BUTTON);
        }, ClawStateMachine.manualOpenState));
    }

    @Override
    public void init() {
        manualConeTimer.start();
    }

    @Override
    public void execute() {

        if (manualConeTimer.hasElapsed(Constants.CLAW.TIME_CLOSE_ON_CONE)) {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.0);
            manualConeTimer.stop();
        } else {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, Constants.CLAW.CLOSE_POWER);
        }

    }

    @Override
    public void exit() {
        manualConeTimer.reset();
    }
    
}
