// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.SwerveModule;

/** Add your docs here. */
public class Swerve {

    public static SwerveDrivePoseEstimator swerveOdometry;
    public static SwerveModule[] mSwerveMods = RobotMap.swerveModules;
    public static WPI_Pigeon2 gyro = RobotMap.gyro;

    //public static RoboLionsPID aprilTagRotationPID = RobotMap.rotationPID;
    public static RoboLionsPID rotationPID = RobotMap.rotationPID;
    public static RoboLionsPID translationPID = RobotMap.translationPID;
    public static RoboLionsPID strafePID = RobotMap.strafePID;

    public static XboxController driverController = RobotMap.driverController;

    public Swerve() {
        rotationPID.initialize(
            0.01,
            0.0,
            0.0,
            2, // Cage Limit degrees/sec 2=without weights, with weights
            2, // Deadband
            0.3, // MaxOutput Degrees/sec 
            true, //enableCage
            false //enableDeadband
        );
        
        translationPID.initialize(
        
            0.15, // Proportional Gain 
            0.0, // Integral Gain
            0.0, // Derivative Gain
            0.0, // Cage Limit
            0.0, // Deadband 
            12,// MaxOutput Volts
            false, //enableCage
            false //enableDeadband
        );

        strafePID.initialize(
            0.15, // Proportional Gain 
            0.0, // Integral Gain
            0.0, // Derivative Gain 
            0.0, // Cage Limit 
            0.0, // Deadband
            12,// MaxOutput Volts 
            false, //enableCage
            false //enableDeadband
        );
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void drive(Translation2d translation, double rotation) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    /** Updates the field relative position of the robot. */
    public void updateSwervePoseEstimator() {
        RobotMap.swerveDrivePoseEstimator.update(
            gyro.getRotation2d(),
            getModulePositions());
    }
}
