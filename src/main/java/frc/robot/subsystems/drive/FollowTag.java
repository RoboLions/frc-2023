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
import frc.robot.lib.PhotonCameraWrapper;
import frc.robot.lib.State;
import frc.robot.lib.Swerve;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

// 1. know current pose
// 2. know target pose
// 3. pass in PID loop

/** Add your docs here. */
public class FollowTag extends State {

    double translationVal = 0;
    double strafeVal = 0;
    double rotationVal = 0;

    double translationCommand = 0;
    double rotationCommand = 0;
    double strafeCommand = 0;

    Pose2d currentPose;
    Pose2d targetPose;

    @Override
    public void init() {
        
        Swerve.gyro.configFactoryDefault();
        RobotMap.swerve.zeroGyro();

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        RobotMap.swerve.resetModulesToAbsolute();

        Swerve.swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, RobotMap.swerve.getYaw(), RobotMap.swerve.getModulePositions());

    }

    @Override
    public void execute() {

        currentPose = RobotMap.swerveDrivePoseEstimator.getEstimatedPosition();
        targetPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0, 0.0));

        translationCommand = Swerve.translationPID.execute(targetPose.getX(), currentPose.getY());
        strafeCommand = Swerve.strafePID.execute(targetPose.getX(), currentPose.getY());
        rotationCommand = Swerve.rotationPID.execute(targetPose.getX(), currentPose.getY());
    
        for(SwerveModule mod : Swerve.mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        System.out.println(translationCommand + strafeCommand + rotationCommand);

        /*RobotMap.swerve.drive(
            new Translation2d(translationCommand, strafeCommand).times(Constants.Swerve.maxSpeed), 
            rotationCommand
        );*/

    }

    @Override
    public void exit() {
        
        RobotMap.swerve.drive(
            new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed), 
            0.0
        );

    }

}