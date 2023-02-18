// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.Swerve;
import frc.robot.lib.RoboLionsPID;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** Add your docs here. */
public class BalanceState extends State{
    public XboxController driverController = RobotMap.driverController;
    public WPI_Pigeon2 gyro = RobotMap.gyro;

    public double pitchP = Constants.balancePitchPID.P;
    public double pitchI = Constants.balancePitchPID.D;
    public double pitchD = Constants.balancePitchPID.I;

    public double rollP = Constants.balancerRollPID.P;
    public double rollI = Constants.balancerRollPID.I;
    public double rollD = Constants.balancerRollPID.D;


    public static RoboLionsPID rollPID = new RoboLionsPID();
    public static RoboLionsPID pitchPID = new RoboLionsPID();


    @Override
    public void init() {
        rollPID.initialize(
            rollD,
            rollI,
            rollD,
            0, 
            5, 
            Constants.Swerve.maxSpeed
        );
        rollPID.enableDeadBand = true;
        rollPID.enableCage = false;
        pitchPID.initialize(
            pitchD,
            pitchI,
            pitchD,
            0, 
            5, 
            Constants.Swerve.maxSpeed
        );
        pitchPID.enableDeadBand = true;
        pitchPID.enableCage = false;
        
    }

    @Override
    public void execute() {
        RobotMap.swerve.drive(
            new Translation2d(
            rollPID.execute(0.0, gyro.getRoll()),
            pitchPID.execute(0.0, gyro.getPitch()))
             .times(Constants.Swerve.maxSpeed), 
            RobotMap.swerve.getPose().getRotation().getDegrees(), 
            false, 
            true
        );

    }

    @Override
    public void build(){
        transitions.add(new Transition(() ->{
            return driverController.getLeftX() > 0.25  || 
            driverController.getLeftX() < -0.25 || 
            driverController.getRightX() > 0.25 ||
            driverController.getRightY() <-0.25;
        }, DrivetrainStateMachine.teleopSwerve));
    }
}
