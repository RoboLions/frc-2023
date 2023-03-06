// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.RoboLionsPID;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Class with methods related to the claw or color sensor */
public class Claw {
    
    private static ColorMatch colorMatcher;

    /* Claw open and close requests */
    public static boolean openRequest = false;
    public static boolean closeRequest = false;

    // public static RoboLionsPID clawPID = new RoboLionsPID();

    public Claw() {
        colorMatcher = new ColorMatch();

        colorMatcher.addColorMatch(Constants.CLAW.CUBE_COLOR);
        colorMatcher.addColorMatch(Constants.CLAW.CONE_COLOR);

        RobotMap.clawMotor.configPeakOutputForward(0.7);
        RobotMap.clawMotor.configPeakOutputReverse(-0.7);

        // clawPID.initialize(
        //     0.1, 0, 0, 0, 5.0, 0.6);
    }

    public Color getColor() {
        // return Constants.CLAW.CONE_COLOR;
        
        ColorMatchResult match = colorMatcher.matchClosestColor(RobotMap.clawColorSensor.getColor());
        if (match.confidence > 0.9) {
            return match.color;
        }
        return null;
    }

    public static void requestClawOpen() {
        Claw.openRequest = true;
        Claw.closeRequest = false;
    }

    public static  void requestClawClosed() {
        Claw.openRequest = false;
        Claw.closeRequest = true;
    }

    public void setClawOpen() {
        RobotMap.clawMotor.set(ControlMode.PercentOutput, -0.8);
    }

    public void setClawClosedCube() {
        RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.8);
    }

    public void setClawClosedCone() {
        RobotMap.clawMotor.set(ControlMode.PercentOutput, 0.8);
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
