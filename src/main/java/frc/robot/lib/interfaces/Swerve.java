// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces;

import java.util.ArrayList;
import java.util.Optional;

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
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.lib.math.Conversions;

/** Class with methods that get used in states of DrivetrainStateMachine */
public class Swerve {

    public static SwerveDrivePoseEstimator swerveOdometry;
    
    public static SwerveModule[] mSwerveMods;

    public static GyroIO gyro;
    private final static GyroIOInputsAutoLogged GyroInputs = new GyroIOInputsAutoLogged();
    //public static PhotonCamera camera;

    private double previousPipelineTimestamp = 0;
    
    private static ArrayList<Pose2d> scoringPoses = new ArrayList<Pose2d>();
    private static ArrayList<Pose2d> loadingStationPoses = new ArrayList<Pose2d>();
    private Pose2d closestPose;
    public static int poseNumber;
    public static AprilTagFieldLayout aprilTagFieldLayout;

    public static Timer timer = new Timer();

    private static double rollOffset = 0.0;
    private static double pitchOffset = 0.0;

    private static boolean leftShiftPrev = false;
    private static boolean rightShiftPrev = false;
    private static boolean bButtonPrev = false;

    public static boolean leftShift = false;
    public static boolean rightShift = false;
    public static boolean bButton = false;

    public Swerve(GyroIO gyro,
    SwerveModuleIO flModuleIO,
    SwerveModuleIO frModuleIO,
    SwerveModuleIO blModuleIO,
    SwerveModuleIO brModuleIO) {
        
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            aprilTagFieldLayout = null;
        }

        this.gyro = gyro;
        mSwerveMods = new SwerveModule[]{
            new SwerveModule(flModuleIO, 0),
            new SwerveModule(frModuleIO, 1),
            new SwerveModule(blModuleIO, 2),
            new SwerveModule(brModuleIO, 3),
        };

        swerveOdometry = new SwerveDrivePoseEstimator(
            Constants.SWERVE.SWERVE_KINEMATICS, 
            RobotMap.gyro.getRotation2d(), 
            getModulePositions(), 
            new Pose2d()
        );
            
        // camera = new PhotonCamera("Arducam_OV9281_USB_Camera"); //HD_USB_Camera
    }

    public static void periodic() {

        for(SwerveModule mod : mSwerveMods){
            mod.period();
        }
        gyro.updateInputs(GyroInputs);

        boolean leftShiftCurr = RobotMap.driverController.getRawButton(Constants.DriverControls.SHIFT_LEFT_BUTTON);
        leftShift = !leftShiftPrev && leftShiftCurr;
        leftShiftPrev = leftShiftCurr;
        boolean rightShiftCurr = RobotMap.driverController.getRawButton(Constants.DriverControls.SHIFT_RIGHT_BUTTON);
        rightShift = !rightShiftPrev && rightShiftCurr;
        rightShiftPrev = rightShiftCurr;

        boolean bButtonCurr = RobotMap.driverController.getRawButton(Constants.DriverControls.TOGGLE_ACCEL_BUTTON);
        bButton = !bButtonPrev && bButtonCurr;
        bButtonPrev = bButtonCurr;

        double starting_x = 0.0;
        Rotation2d rotation = Rotation2d.fromDegrees(0.0);
        Pose2d starting_loading_station = new Pose2d();
        Rotation2d loading_station_rotation = Rotation2d.fromDegrees(0.0);

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            starting_x = Constants.TargetPoses.RED_TRANSPOSE_DISTANCE - Constants.TargetPoses.BLUE_SCORING_X;
            rotation = Rotation2d.fromDegrees(0.0);
            starting_loading_station = new Pose2d(
                Constants.TargetPoses.RED_TRANSPOSE_DISTANCE - Constants.TargetPoses.BLUE_SUBSTATION_X, 
                Constants.TargetPoses.BLUE_SUBSTATION_Y, 
                loading_station_rotation = Rotation2d.fromDegrees(180.0)
            );
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            starting_x = Constants.TargetPoses.BLUE_SCORING_X;
            rotation = Rotation2d.fromDegrees(180.0);
            starting_loading_station = new Pose2d(
                Constants.TargetPoses.BLUE_SUBSTATION_X,
                Constants.TargetPoses.BLUE_SUBSTATION_Y,
                loading_station_rotation = Rotation2d.fromDegrees(0.0)
            );
        }

        scoringPoses.clear();
        loadingStationPoses.clear();

        for (int i = 0; i < 9; i++) {
            scoringPoses.add(new Pose2d(
                starting_x,
                Constants.TargetPoses.BLUE_SCORING_Y + i * Constants.TargetPoses.SCORING_SPACING,
                rotation
            ));
        }

        for (int i = 0; i < 2; i++) {
            loadingStationPoses.add(new Pose2d(
                starting_loading_station.getX(),
                Constants.TargetPoses.BLUE_SUBSTATION_Y - i * Constants.TargetPoses.SUBSTATION_SPACING,
                loading_station_rotation
            ));
        }
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
            mod.io.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, mod.getState());
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SWERVE.MAX_SPEED);
        
        for(SwerveModule mod : mSwerveMods){
            mod.io.setDesiredState(desiredStates[mod.moduleNumber], false, mod.getState());
        }
    }
    
    private void updateSwervePoseAprilTags() {
        // Update pose estimator with the best visible target
        // var pipelineResult = camera.getLatestResult();
        // var resultTimestamp = pipelineResult.getTimestampSeconds();

        // if (resultTimestamp == previousPipelineTimestamp) {
        //     return;
        // }
        // if (!pipelineResult.hasTargets()) {
        //     return;
        // }

        // previousPipelineTimestamp = resultTimestamp;
        // var target = pipelineResult.getBestTarget();
        // // System.out.println(target);
        // var fiducialId = target.getFiducialId();
        
        // if (fiducialId <= 0 || fiducialId > 8) {
        //     return;
        // }

        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        // Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);

        // if (target.getPoseAmbiguity() > 0.1) {
        //     return;
        // }
        // if (!tagPose.isPresent()) {
        //     return;
        // }

        
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

    private Pose2d lastLLBotPose2d = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0.0));

    private void updateSwervePoseLimelight() {
        // double tl = LimelightHelpers.getLatency_Pipeline(Constants.LIMELIGHT.NAME);
        // double tc = LimelightHelpers.getLatency_Capture(Constants.LIMELIGHT.NAME);
        LimelightResults results = LimelightHelpers.getLatestResults(Constants.LIMELIGHT.NAME);
        var num_targets = results.targetingResults.targets_Fiducials.length;
        Pose2d botPose2d = LimelightHelpers.toPose2D(results.targetingResults.botpose_wpiblue);
        double tl = results.targetingResults.latency_pipeline;
        double tc = results.targetingResults.latency_capture;

        if (Math.abs(results.targetingResults.botpose_wpiblue[0]) < 1.0E-10 && 
            Math.abs(results.targetingResults.botpose_wpiblue[1]) < 1.0E-10 && 
            Math.abs(results.targetingResults.botpose_wpiblue[2]) < 1.0E-10 && 
            Math.abs(results.targetingResults.botpose_wpiblue[3]) < 1.0E-10 && 
            Math.abs(results.targetingResults.botpose_wpiblue[4]) < 1.0E-10 && 
            Math.abs(results.targetingResults.botpose_wpiblue[5]) < 1.0E-10) {
            return;
        }

        if (num_targets != 1) {
            return;
        }

        if (botPose2d.getTranslation().getDistance(lastLLBotPose2d.getTranslation()) > 1.0) {
            lastLLBotPose2d = botPose2d;
            return;
        }
        lastLLBotPose2d = botPose2d;

        if ((int)LimelightHelpers.getFiducialID(Constants.LIMELIGHT.NAME) == -1.0) {
            return;
        }

        if (LimelightHelpers.getTargetPose3d_CameraSpace(Constants.LIMELIGHT.NAME).getZ() > 2.5) {
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
            double absolutePosition = Conversions.degreesToFalcon(mod.getCanCoder().getDegrees() - mod.angleOffset.getDegrees(), Constants.SWERVE.ANGLE_GEAR_RATIO);
            mod.io.resetToAbsolute(absolutePosition);
        }
    }
    
    private void updateSwervePoseKinematics() {
        // Update pose estimator with drivetrain sensors
        swerveOdometry.update(
            RobotMap.gyro.getRotation2d(),
            RobotMap.swerve.getModulePositions());
    }

    public void updatePoses() {
        updateSwervePoseKinematics();
        if (!DriverStation.isAutonomous()) {
            updateSwervePoseLimelight();
            //updateSwervePoseAprilTags();
        }
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
        double shortestDistance = 1000.0;

        // compare the distance between our current pose and loading station poses
        for (int i = 0; i < loadingStationPoses.size(); i++) {
            System.out.println("Loading station pose " + i + ": " + loadingStationPoses.get(i));
            double temp_distance = currentPose.getTranslation().getDistance(loadingStationPoses.get(i).getTranslation());
            if (temp_distance < shortestDistance) {
                shortestDistance = temp_distance;
                closestPose = loadingStationPoses.get(i);
                System.out.println("Setting closest pose to: " + closestPose);
                // pose number = -1 still, no pose shifting at the substation
            }
        }

        // compare the distance between current pose and scoring poses
        for (int i = 0; i < scoringPoses.size(); i++) {
            double temp_distance = currentPose.getTranslation().getDistance(scoringPoses.get(i).getTranslation());
            System.out.println("Scoring pose " + i + ": " + scoringPoses.get(i));
            /* the shortest distance is currently the distance to loading station pose 0 or 1
            if the distance from current pose to the scoring pose is smaller than 
            distance to closest loading station pose, make this our closest pose */
            if (temp_distance < shortestDistance) {
                shortestDistance = temp_distance;
                closestPose = scoringPoses.get(i);
                System.out.println("Setting closest pose to: " + closestPose);
                poseNumber = i;
            }
        }

        System.out.println("Shortest distance: " + shortestDistance);
        System.out.println("Closest pose: " + closestPose);
        System.out.println("Pose #: " + poseNumber);

        return shortestDistance;
    }

    public void shiftPose(boolean increase) {
        if (poseNumber == -1) {
            return;
        }
        poseNumber = increase ? 
            poseNumber + 1 :
            poseNumber - 1;
        poseNumber = Math.max(Math.min(poseNumber, 8), 0);
        closestPose = scoringPoses.get(poseNumber);
        System.out.println("Shifting to pose " + closestPose);
    }

    public void shiftPoseRight() {
        shiftPose(DriverStation.getAlliance() == DriverStation.Alliance.Red);
    }

    public void shiftPoseLeft() {
        shiftPose(DriverStation.getAlliance() == DriverStation.Alliance.Blue);
    }

    public Pose2d getClosestPose() {
        return closestPose;
    }
}