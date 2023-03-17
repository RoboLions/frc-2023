// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import java.util.ArrayList;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotMap;

/** Class with methods that get used in states of DrivetrainStateMachine */
public class Swerve {

    public static SwerveDrivePoseEstimator swerveOdometry;
    public static SwerveModule[] mSwerveMods;

    public static PhotonCamera camera;

    private double previousPipelineTimestamp = 0;
    
    private static ArrayList<Pose2d> scoringPoses = new ArrayList<Pose2d>();
    private static Pose2d loadingStation;
    private Pose2d closestPose;
    public static int poseNumber;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public static Timer timer = new Timer();

    private static double rollOffset = 0.0;
    private static double pitchOffset = 0.0;

    private static boolean leftShiftPrev = false;
    private static boolean rightShiftPrev = false;

    public static boolean leftShift = false;
    public static boolean rightShift = false;

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
            new Pose2d()
        );
            
        camera = new PhotonCamera("Arducam_OV9281_USB_Camera"); //HD_USB_Camera

        double starting_x = 0.0;
        Rotation2d rotation = Rotation2d.fromDegrees(0.0);

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            starting_x = Constants.TargetPoses.BLUE_SCORING_X + 100.0;
            rotation = Rotation2d.fromDegrees(0.0);
            loadingStation = new Pose2d(
                Constants.TargetPoses.BLUE_SUBSTATION_X, 
                Constants.TargetPoses.BLUE_SUBSTATION_Y, 
                Rotation2d.fromDegrees(180.0)
            );
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            starting_x = Constants.TargetPoses.BLUE_SCORING_X;
            rotation = Rotation2d.fromDegrees(180.0);
            loadingStation = new Pose2d(
                Constants.TargetPoses.BLUE_SUBSTATION_X,
                Constants.TargetPoses.BLUE_SUBSTATION_Y,
                Rotation2d.fromDegrees(0.0)
            );
        }

        for (int i = 0; i < 9; i++) {
            scoringPoses.add(new Pose2d(
                starting_x,
                Constants.TargetPoses.BLUE_SCORING_Y + i * Constants.TargetPoses.SCORING_SPACING,
                rotation
            ));
        }

    }

    public static void periodic() {
        boolean leftShiftCurr = RobotMap.driverController.getRawButton(Constants.DriverControls.SHIFT_LEFT_BUTTON);
        leftShift = !leftShiftPrev && leftShiftCurr;
        leftShiftPrev = leftShiftCurr;
        boolean rightShiftCurr = RobotMap.driverController.getRawButton(Constants.DriverControls.SHIFT_RIGHT_BUTTON);
        rightShift = !rightShiftPrev && rightShiftCurr;
        rightShiftPrev = rightShiftCurr;
    }

    public static void zeroRoll() {
        rollOffset = RobotMap.gyro.getRoll();
    }

    public static double getRoll() {
        return RobotMap.gyro.getRoll() - rollOffset;
    }

    public static void zeroPitch() {
        pitchOffset = RobotMap.gyro.getPitch();
    }

    public static double getPitch() {
        return RobotMap.gyro.getPitch() - pitchOffset;
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
        // System.out.println(target);
        var fiducialId = target.getFiducialId();
        
        if (fiducialId <= 0 || fiducialId > 8) {
            return;
        }

        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);

        if (target.getPoseAmbiguity() > 0.1) {
            return;
        }
        if (!tagPose.isPresent()) {
            return;
        }

        
        // FieldObject2d tagPoseDisplay = RobotMap.Field2d.getObject("tagPose");
        // tagPoseDisplay.setPose(tagPose.get().toPose2d());
        // var targetPose = tagPose.get();
        // Transform3d camToTarget = target.getBestCameraToTarget();
        // System.out.println(camToTarget);
        // Pose2d camPose = new Pose2d(
        //     targetPose.getX() + camToTarget.getX(), 
        //     targetPose.getY() + camToTarget.getY(), 
        //     targetPose.getRotation().toRotation2d().plus(camToTarget.getRotation().toRotation2d())
        // );
        // // Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
        // FieldObject2d camPoseDisplay = RobotMap.Field2d.getObject("camPose");
        // camPoseDisplay.setPose(camPose);
        // var visionMeasurement = camPose;
        // // var visionMeasurement = camPose.transformBy(Constants.PhotonConstants.ROBOT_TO_CAM);
        // FieldObject2d visionMeasurementDisplay = RobotMap.Field2d.getObject("visionMeasurement");
        // visionMeasurementDisplay.setPose(visionMeasurement);
        // swerveOdometry.addVisionMeasurement(visionMeasurement, resultTimestamp);
    }

    private void updateSwervePoseLimelight() {
        timer.reset();
        timer.start();
        Pose2d botPose2d = LimelightHelpers.getBotPose2d_wpiBlue(Constants.LIMELIGHT.NAME);
        double tl = LimelightHelpers.getLatency_Pipeline(Constants.LIMELIGHT.NAME);
        double tc = LimelightHelpers.getLatency_Capture(Constants.LIMELIGHT.NAME);
        timer.stop();

        if ((int)LimelightHelpers.getFiducialID(Constants.LIMELIGHT.NAME) == -1.0) {
            return;
        }

        // SmartDashboard.putNumber("LL pose data X", botPose2d.getTranslation().getX());
        // SmartDashboard.putNumber("LL pose data Y", botPose2d.getTranslation().getY());
        // SmartDashboard.putNumber("LL pose data rotation", botPose2d.getRotation().getDegrees());
        // SmartDashboard.putNumber("LL delay", timer.get() / 1000.0);

        swerveOdometry.addVisionMeasurement(botPose2d, 
        Timer.getFPGATimestamp() - 
            (tl/1000.0) - 
            (tc/1000.0)
        );
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
        updateSwervePoseLimelight();
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

        for (int i = 0; i < scoringPoses.size(); i++) {
            double temp_distance = currentPose.getTranslation().getDistance(scoringPoses.get(i).getTranslation());
            if (temp_distance < shortestDistance) {
                shortestDistance = temp_distance;
                closestPose = scoringPoses.get(i);
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
            poseNumber - 1 :
            poseNumber + 1;
        poseNumber = Math.max(Math.min(poseNumber, 8), 0);
        closestPose = scoringPoses.get(poseNumber);
        System.out.println("Shifting to pose " + closestPose);
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