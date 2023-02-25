// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Class with methods related to the claw or color sensor */
public class Claw {
    
    private static ColorMatch colorMatcher;
    
    public RoboLionsPID clawPID = new RoboLionsPID();

    /* Claw open and close requests */
    public static boolean openRequest = false;
    public static boolean closeRequest = false;

    public Claw() {
        colorMatcher = new ColorMatch();
        RobotMap.clawMotor.setNeutralMode(NeutralMode.Brake);
        colorMatcher.addColorMatch(Constants.CLAW.CUBE_COLOR);
        colorMatcher.addColorMatch(Constants.CLAW.CONE_COLOR);

        clawPID.initialize(
                        0.0, // Proportional Gain 0.02
                        0.0, // Integral Gain .311
                        0.0, // Derivative Gain
                        1, // Cage Limit
                        1, // Deadband
                        2 // MaxOutput hard deadband as to what the maximum possible command is
        );
    }

    public Color updateDetectedColor() {
        return RobotMap.clawColorSensor.getColor();
    }

    public Color getColor() {
        return Constants.CLAW.CONE_COLOR;
        
        /*ColorMatchResult match = colorMatcher.matchClosestColor(RobotMap.clawColorSensor.getColor());
        if (match.confidence > 0.9) {
            return match.color;
        }
        return null;*/
    }

    public int getClawEncoder() {
        return RobotMap.clawEncoder.get();
    }
 
    public boolean getDirection() {
        return RobotMap.clawEncoder.getDirection();
    }

    public void moveClawToPosition(double target, double feedback) {
        double claw_cmd = clawPID.execute(target, feedback);
    
        RobotMap.clawMotor.set(ControlMode.PercentOutput, claw_cmd);
      }

    public void setClawOpen() {
        // moveClawToPosition(Constants.CLAW.OPEN_POSITION, getClawEncoder());
        
        RobotMap.clawMotor.set(ControlMode.PercentOutput, -0.15);
    }

    public void setClawClosedCube() {
        // moveClawToPosition(Constants.CLAW.CLOSED_CUBE_POSITION, getClawEncoder());
        
        RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.15);
    }

    public void setClawClosedCone() {
        // moveClawToPosition(Constants.CLAW.CLOSED_CONE_POSITION, getClawEncoder());
        
        RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.65);
    }

    public boolean isClosed() {
        return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCone || 
            RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCube;
    }

    public boolean isClosing() {
        return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closingCone || 
            RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closingCube;
    }

    public boolean isOpen() {
        return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.openState;
    }

    public boolean isOpening() {
        return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.openingState;
    }
}
