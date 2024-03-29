// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.LED.LEDStateMachine;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Class with methods related to the claw or color sensor */
public class Intake {

    public Intake() {
        RobotMap.intakeMotor.configPeakOutputForward(1.0);
        RobotMap.intakeMotor.configPeakOutputReverse(-1.0);
        RobotMap.intakeMotor.setInverted(true);
    }

    public void runIntake() {
        if (RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState) {
            RobotMap.intakeMotor.set(ControlMode.PercentOutput, -1.0 * Constants.INTAKE.INTAKE_POWER);
        } else {
            RobotMap.intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE.INTAKE_POWER);
        }
    }

    public void runOuttake() {
        if (RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState) {
            RobotMap.intakeMotor.set(ControlMode.PercentOutput, -1.0 * Constants.INTAKE.OUTTAKE_POWER);
        } else if (RobotMap.ledStateMachine.getCurrentState() == LEDStateMachine.coneLEDState && RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.scoreMidState) {
            RobotMap.intakeMotor.set(ControlMode.PercentOutput, -1.0 * Constants.INTAKE.OUTTAKE_POWER * 1.4);
        } else {
            RobotMap.intakeMotor.set(ControlMode.PercentOutput, Constants.INTAKE.OUTTAKE_POWER);
        }
    }
}