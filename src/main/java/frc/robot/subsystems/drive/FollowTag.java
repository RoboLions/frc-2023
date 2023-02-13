// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SwerveModule;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.lib.states.Swerve;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

// 1. know current pose
// 2. know target pose
// 3. pass in PID loop

/** Add your docs here. */
public class FollowTag extends State {

    private static XboxController driverController = RobotMap.driverController;

    double translationVal = 0;
    double strafeVal = 0;
    double rotationVal = 0;

    double translationCommand = 0;
    double rotationCommand = 0;
    double strafeCommand = 0;

    Pose2d currentPose;
    Pose2d targetPose;

    double currentYaw;

    @Override
    public void build() {
        transitions.add(new Transition(() -> {
            return driverController.getBButton(); 
        }, DrivetrainStateMachine.teleopState));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {

        currentPose = Swerve.swerveOdometry.getEstimatedPosition();
        targetPose = new Pose2d(3.5, 3.0, new Rotation2d(0.0));
        currentYaw = RobotMap.swerve.getYaw().getDegrees();
        if (currentYaw < 0.0) {
            currentYaw = currentYaw * -1.0;
        }
        if (currentYaw >= 360.0) {
            currentYaw = currentYaw % 360;
        }

        translationCommand = Swerve.translationPID.execute(targetPose.getX(), currentPose.getX());
        strafeCommand = Swerve.strafePID.execute(targetPose.getY(), currentPose.getY());
        //rotationCommand = Swerve.rotationPID.execute(targetPose.getRotation().getDegrees(), currentYaw);

        if (translationCommand < 0.01 && translationCommand > -0.01) {
            translationCommand = 0.0;
        }
        if (strafeCommand < 0.01 && strafeCommand > -0.01) {
            strafeCommand = 0.0;
        }
        /*if (rotationCommand < 0.01 && rotationCommand > -0.01) {
            rotationCommand = 0.0;
        }*/

        //System.out.println("R: " + rotationCommand);

        RobotMap.swerve.drive(
            new Translation2d(-translationCommand, -strafeCommand).times(8.0), 
            0.0
        );

    }

    @Override
    public void exit() {
        
        RobotMap.swerve.drive(
            new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed), 
            0.0
        );

    }

}