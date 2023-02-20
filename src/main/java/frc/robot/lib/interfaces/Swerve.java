// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import java.util.Optional;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotMap;

/** Class with methods that get used in states of DrivetrainStateMachine */
public class Swerve {

    public static SwerveDrivePoseEstimator swerveOdometry;
    public static SwerveModule[] mSwerveMods;

    public static PhotonCamera camera;

    private double previousPipelineTimestamp = 0;
    
    private static Pose2d[] scoringPoses;
    private static Pose2d loadingStation;
    private Pose2d closestPose;
    public static int poseNumber;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public Swerve() {
        
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            aprilTagFieldLayout = null;
        }

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SWERVE.Mod0.constants),
            new SwerveModule(1, Constants.SWERVE.Mod1.constants),
            new SwerveModule(2, Constants.SWERVE.Mod2.constants),
            new SwerveModule(3, Constants.SWERVE.Mod3.constants)
        };

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.SWERVE.SWERVE_KINEMATICS, 
            RobotMap.gyro.getRotation2d(), 
            getModulePositions(), 
            new Pose2d() // TODO: should this be our initial pose instead of new Pose2d()?
        );
            
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera"); //HD_USB_Camera

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            scoringPoses = Constants.TargetPoses.RED_SCORING_POSES;
            loadingStation = Constants.TargetPoses.RED_LOADING_STATION;
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            scoringPoses = Constants.TargetPoses.BLUE_SCORING_POSES;
            loadingStation = Constants.TargetPoses.BLUE_LOADING_STATION;
        }
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.SWERVE.SWERVE_KINEMATICS.toSwerveModuleStates(
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
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SWERVE.MAX_SPEED);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SWERVE.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
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
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);

        if (target.getPoseAmbiguity() > 0.2) {
            return;
        }
        if (!tagPose.isPresent()) {
            return;
        }

        var targetPose = tagPose.get();
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
        var visionMeasurement = camPose.transformBy(Constants.PhotonConstants.ROBOT_TO_CAM);
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
        return (Constants.SWERVE.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - RobotMap.gyro.getYaw()) : Rotation2d.fromDegrees(RobotMap.gyro.getYaw());
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

    public double getPoseDistance(Pose2d currentPose, Pose2d targetPose) {
        return currentPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    public double setClosestPose(Pose2d currentPose) {

        // get the closest pose
        // get distance between current pose and target pose of every target pose 
        // for the pose with shortest distance, make that the closest pose
        poseNumber = -1;
        closestPose = loadingStation;
        double shortestDistance = currentPose.getTranslation().getDistance(loadingStation.getTranslation());

        for (int i = 0; i < scoringPoses.length; i++) {
            double temp_distance = currentPose.getTranslation().getDistance(scoringPoses[i].getTranslation());
            if (temp_distance < shortestDistance) {
                shortestDistance = temp_distance;
                closestPose = scoringPoses[i];
                poseNumber = i;
            }
        }

        return shortestDistance;
    }

    public void shiftPose(boolean go_right) {
        if (poseNumber == -1) {
            return;
        }
        poseNumber = go_right ? 
            poseNumber + 1 :
            poseNumber - 1;
        poseNumber = Math.max(Math.min(poseNumber, 9), 0);
        closestPose = scoringPoses[poseNumber];
    }

    public void shiftPoseRight() {
        shiftPose(true);
    }

    public void shiftPoseLeft() {
        shiftPose(false);
    }

    public Pose2d getClosestPose() {
        return closestPose;
    }
}