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
public class ManualOpenState extends State {

    private Timer manualOpenTimer = new Timer();
    
    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.MANUAL_CLOSE_CUBE_BUTTON);
        }, ClawStateMachine.manualCubeState));

        transitions.add(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.MANUAL_CLOSE_CONE_BUTTON);
        }, ClawStateMachine.manualConeState));
    }

    @Override
    public void init() {

    }

    @Override
    public void execute() {

        if (manualOpenTimer.hasElapsed(Constants.CLAW.TIME_OPEN_CLAW)) {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.0);
            manualOpenTimer.stop();
        } else {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, Constants.CLAW.OPEN_POWER);
        }

    }

    @Override
    public void exit() {
        manualOpenTimer.reset();
    }
}
