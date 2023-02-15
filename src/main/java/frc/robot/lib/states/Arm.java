// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.states;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotMap;

/** Add your docs here. */
public class Arm {

    private Timer timer = new Timer();

    public Arm() {
        RobotMap.armFirstStage.setNeutralMode(NeutralMode.Brake);
        RobotMap.armSecondStage.setNeutralMode(NeutralMode.Brake);
        RobotMap.wrist.setNeutralMode(NeutralMode.Brake);

        RobotMap.armFirstStage.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.armSecondStage.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);
        RobotMap.wrist.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 10);

        RobotMap.armFirstStage.configNominalOutputForward(0, 10);
        RobotMap.armFirstStage.configNominalOutputReverse(0, 10);
        RobotMap.armFirstStage.configPeakOutputForward(1, 10);
        RobotMap.armFirstStage.configPeakOutputReverse(-1, 10);
        RobotMap.armFirstStage.configNeutralDeadband(0.001, 10);

        RobotMap.armSecondStage.configNominalOutputForward(0, 10);
        RobotMap.armSecondStage.configNominalOutputReverse(0, 10);
        RobotMap.armSecondStage.configPeakOutputForward(1, 10);
        RobotMap.armSecondStage.configPeakOutputReverse(-1, 10);
        RobotMap.armSecondStage.configNeutralDeadband(0.001, 10);

        RobotMap.wrist.configNominalOutputForward(0, 10);
        RobotMap.wrist.configNominalOutputReverse(0, 10);
        RobotMap.wrist.configPeakOutputForward(1, 10);
        RobotMap.wrist.configPeakOutputReverse(-1, 10);
        RobotMap.wrist.configNeutralDeadband(0.001, 10);

        RobotMap.armFirstStage.configAllowableClosedloopError(0, 0, 10);
        RobotMap.armSecondStage.configAllowableClosedloopError(0, 0, 10);
        RobotMap.wrist.configAllowableClosedloopError(0, 0, 10);
    }

    /*public boolean getBaseSensor() {
        return false;
    }*/

    public void setIdle() {
        RobotMap.armFirstStage.set(TalonFXControlMode.Position, 0.0);
        RobotMap.armSecondStage.set(TalonFXControlMode.Position, 0.0);
        RobotMap.wrist.set(TalonFXControlMode.Position, 0.0);
    }

    public void moveArmPosition(double firstStage, double secondStage, double wrist) {
        RobotMap.armFirstStage.set(TalonFXControlMode.Position, firstStage);
        RobotMap.armSecondStage.set(TalonFXControlMode.Position, secondStage);
        RobotMap.wrist.set(TalonFXControlMode.Position, wrist);
    }

    // method to check if arm has arrived at its position
    public boolean getArrived(double allowance, double time) {

        // TODO: test and change error allowance
        if (Math.abs(RobotMap.armFirstStage.getClosedLoopError()) <= Math.abs(allowance) && 
            Math.abs(RobotMap.armSecondStage.getClosedLoopError()) <= Math.abs(allowance) &&
            Math.abs(RobotMap.wrist.getClosedLoopError()) <= Math.abs(allowance)) { 
            timer.start();
            if (timer.hasElapsed(time)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }
        return false;
    }

    public boolean getClawClosed() {

        // assume claw is closed after some # of counts
        if (RobotMap.claw.getColor() == RobotMap.cubeColor) {
            timer.start();
            if (timer.hasElapsed(Constants.Claw.clawClosedCubeTime)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }

        if (RobotMap.claw.getColor() == RobotMap.coneColor) {
            timer.start();
            if (timer.hasElapsed(Constants.Claw.clawClosedConeTime)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }

        return false;
    }

    // assume claw is open after some # of counts
    public boolean getClawOpen() {
        if (!RobotMap.claw.getColor() == RobotMap.coneColor || 
            !RobotMap.claw.getColor() == RobotMap.cubeColor) {
            timer.start();
            if (timer.hasElapsed(Constants.Claw.clawClosedConeTime)) {
                timer.stop();
                timer.reset();
                return true;
            }
        }
        return false;
    }
}
