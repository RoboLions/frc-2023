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
public class ClosingCube extends State {
    
    private Timer closingCubeTimer = new Timer();
    
    @Override
    public void build() {
        // claw is now closed on a cube after x seconds
        transitions.add(new Transition(() -> {
            return closingCubeTimer.hasElapsed(Constants.CLAW.TIME_CLOSE_ON_CUBE + 0.1);
        }, ClawStateMachine.closedCube));
    }

    @Override
    public void init() {
        closingCubeTimer.start();
    }

    @Override
    public void execute() {
        
        SmartDashboard.putNumber("Claw closing on cube timer", closingCubeTimer.get());

        // apply power to claw motor long enough to close on the cube
        if (closingCubeTimer.hasElapsed(Constants.CLAW.TIME_CLOSE_ON_CUBE)) {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.0);
        } else {
            RobotMap.clawMotor.set(ControlMode.PercentOutput, Constants.CLAW.CLOSE_POWER);
        }
        
    }

    @Override
    public void exit() {
        closingCubeTimer.stop();
        closingCubeTimer.reset();
    }
    
}
