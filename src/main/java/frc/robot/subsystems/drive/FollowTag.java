// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SwerveModule;
import frc.robot.lib.State;
import frc.robot.lib.Swerve;

/** Add your docs here. */
public class FollowTag extends State {

    double translationVal = 0;
    double strafeVal = 0;
    double rotationVal;

    //final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(28.0);
    //final double TARGET_HEIGHT_METERS = Units.feetToMeters(5.9);
    // Angle between horizontal and the camera.
    //final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(90);

    // How far from the target we want to be
    //final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // PID constants should be tuned per robot
    /*final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);*/

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

        var result = RobotMap.camera.getLatestResult();

        if (result.hasTargets()) {
            // Calculate angular turn power
            // -1.0 required to ensure positive PID controller effort _increases_ yaw
            rotationVal = 1.0 * Swerve.aprilTagRotationPID.execute(0.0, result.getBestTarget().getYaw());
            //System.out.println(rotationVal);
        } else {
            // If we have no targets, stay still.
            rotationVal = 0;
        }
            
        Swerve.swerveOdometry.update(RobotMap.swerve.getYaw(), RobotMap.swerve.getModulePositions());  

        for(SwerveModule mod : Swerve.mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        RobotMap.swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity
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