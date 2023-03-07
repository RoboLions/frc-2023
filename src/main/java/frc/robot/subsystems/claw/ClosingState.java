// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.claw;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Claw;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class ClosingState extends State {
    
    private PIDController controller = new PIDController(
        0.01, 0.0, 0.0
    );

    private Timer timer = new Timer();
    
    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return Claw.getArrived(Constants.CLAW.ALLOWANCE, Constants.CLAW.TIME, Constants.CLAW.CLOSED_POSITION);
        }, ClawStateMachine.closedState));

        transitions.add(new Transition(() -> {
            return timer.hasElapsed(Constants.CLAW.TIMEOUT);
        }, ClawStateMachine.closedState));
    }

    @Override
    public void init() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double command = controller.calculate(RobotMap.clawEncoder.get(), Constants.CLAW.CLOSED_POSITION);
        SmartDashboard.putNumber("Closing command", command);
        RobotMap.clawMotor.set(ControlMode.PercentOutput, command);
    }

    @Override
    public void exit() {
        
    }
}
