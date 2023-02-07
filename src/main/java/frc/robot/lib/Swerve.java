// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
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
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Add your docs here. */
public class Swerve {

    public static SwerveDrivePoseEstimator swerveOdometry;
    public static SwerveModule[] mSwerveMods;

    public static RoboLionsPID rotationPID = new RoboLionsPID();
    public static RoboLionsPID translationPID = new RoboLionsPID();
    public static RoboLionsPID strafePID = new RoboLionsPID();
    
    public static PhotonCamera camera;

    private double previousPipelineTimestamp = 0;

    public Swerve() {
    
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, RobotMap.gyro.getRotation2d(), getModulePositions(), new Pose2d());
            
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera"); //HD_USB_Camera

        rotationPID.initialize(
            0.01,
            0.0,
            0.0,
            2.0, // Cage Limit degrees/sec
            2.0, // Deadband
            0.3 // MaxOutput Degrees/sec
        );
        
        translationPID.initialize(
            0.15, // Proportional Gain 
            0.0, // Integral Gain
            0.0, // Derivative Gain
            12 // MaxOutput Volts
        );

        strafePID.initialize(
            0.15, // Proportional Gain 
            0.0, // Integral Gain
            0.0, // Derivative Gain 
            12 // MaxOutput Volts
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

    public PhotonCamera getCamera() {
        return camera;
    }
    
    private void updateSwervePoseAprilTags() {
        // Update pose estimator with the best visible target
        var pipelineResult = camera.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();

        if (resultTimestamp == previousPipelineTimestamp) {
            return;
        }
        if (!pipelineResult.hasTargets()) {
            return;
        }

        previousPipelineTimestamp = resultTimestamp;
        var target = pipelineResult.getBestTarget();
        var fiducialId = target.getFiducialId();
        
        if (fiducialId <= 0 || fiducialId > 8) {
            return;
        }

        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        Optional<Pose3d> tagPose = RobotMap.aprilTagFieldLayout.getTagPose(fiducialId);

        if (target.getPoseAmbiguity() > 0.2) {
            return;
        }
        if (!tagPose.isPresent()) {
            return;
        }

        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
        var visionMeasurement = camPose.transformBy(Constants.PhotonConstants.robotToCam);
        swerveOdometry.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
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
        RobotMap.gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - RobotMap.gyro.getYaw()) : Rotation2d.fromDegrees(RobotMap.gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    
    private void updateSwervePoseKinematics() {
        // Update pose estimator with drivetrain sensors
        swerveOdometry.update(
            RobotMap.gyro.getRotation2d(),
            RobotMap.swerve.getModulePositions());
    }

    public void updatePoses() {
        updateSwervePoseAprilTags();
        updateSwervePoseKinematics();
    }

    /** Updates the field relative position of the robot. */
    public void updateSwervePoseEstimator() {
        swerveOdometry.update(
            RobotMap.gyro.getRotation2d(),
            getModulePositions());
    }
}
