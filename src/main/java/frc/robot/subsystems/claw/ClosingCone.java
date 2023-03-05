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
public class ClosingCone extends State {
    
    private Timer closingConeTimer = new Timer();
    
    @Override
    public void build() {
        // claw is now closed on a cone after x seconds
        transitions.add(new Transition(() -> {
            return closingConeTimer.hasElapsed(Constants.CLAW.TIME_CLOSE_ON_CONE);
        }, ClawStateMachine.closedCone));
    }

    @Override
    public void init() {
        closingConeTimer.start();
        RobotMap.clawMotor.set(ControlMode.PercentOutput, Constants.CLAW.CLOSE_POWER);
    }

    @Override
    public void execute() {
        
       //  SmartDashboard.putNumber("Claw closing on cone timer", closingConeTimer.get());

    }

    @Override
    public void exit() {
        closingConeTimer.stop();
        closingConeTimer.reset();
    }
}
