// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.RobotMap;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonCameraWrapper {
    public PhotonCamera photonCamera = RobotMap.camera;
    public PhotonPoseEstimator photonPoseEstimator = RobotMap.photonPoseEstimator;

    private double previousPipelineTimestamp = 0;

    public PhotonCameraWrapper() {}

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    public void updatePoses() {
        // Update pose estimator with the best visible target
        var pipelineResult = photonCamera.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();
        if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
        previousPipelineTimestamp = resultTimestamp;
        var target = pipelineResult.getBestTarget();
        var fiducialId = target.getFiducialId();
        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        Optional<Pose3d> tagPose = RobotMap.aprilTagFieldLayout == null ? Optional.empty() : RobotMap.aprilTagFieldLayout.getTagPose(fiducialId);
        if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
            var targetPose = tagPose.get();
            Transform3d camToTarget = target.getBestCameraToTarget();
            Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

            var visionMeasurement = camPose.transformBy(Constants.PhotonConstants.robotToCam);
            RobotMap.swerveDrivePoseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
        }
        }
        // Update pose estimator with drivetrain sensors
        RobotMap.swerveDrivePoseEstimator.update(
            RobotMap.gyro.getRotation2d(),
            RobotMap.swerve.getModulePositions());
    }
}