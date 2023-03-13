// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** State to auto-align to a known position on the field */
public class FollowTag extends State {

    private final ProfiledPIDController xController =
        new ProfiledPIDController(
            1.0, 0.0, 0.0, 
            new TrapezoidProfile.Constraints(1.0, 1.0)
    );

    private final ProfiledPIDController yController =
        new ProfiledPIDController(
            1.0, 0.0, 0.0, 
            new TrapezoidProfile.Constraints(1.0, 1.0)
    );

    private final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            1.0, 0.0, 0.0, 
            new TrapezoidProfile.Constraints(1.0, 1.0)
    );

    @Override
    public void build() {
        // if driver joysticks are engaged, transition to teleop state
        addTransition(new Transition(() -> {
            return Math.abs(RobotMap.driverController.getLeftX()) > Constants.STICK_DEADBAND || 
                Math.abs(RobotMap.driverController.getLeftY()) > Constants.STICK_DEADBAND || 
                Math.abs(RobotMap.driverController.getRightX()) > Constants.STICK_DEADBAND ||
                Math.abs(RobotMap.driverController.getRightY()) > Constants.STICK_DEADBAND;
        }, DrivetrainStateMachine.teleopSwerve));
    }

    @Override
    public void init() {
        Pose2d currentPose = Swerve.swerveOdometry.getEstimatedPosition();
        RobotMap.swerve.setClosestPose(currentPose);
    }

    @Override
    public void execute() {
        if (RobotMap.driverController.getRawButton(Constants.DriverControls.SHIFT_LEFT_BUTTON)) {
            RobotMap.swerve.shiftPoseLeft();
        } else if (RobotMap.driverController.getRawButton(Constants.DriverControls.SHIFT_RIGHT_BUTTON)) {
            RobotMap.swerve.shiftPoseRight();
        }

        double targetPoseX = RobotMap.swerve.getClosestPose().getX();
        double targetPoseY = RobotMap.swerve.getClosestPose().getY();
        double targetPoseRotation = RobotMap.swerve.getClosestPose().getRotation().getDegrees();

        double currentPoseX = Swerve.swerveOdometry.getEstimatedPosition().getX();
        double currentPoseY = Swerve.swerveOdometry.getEstimatedPosition().getY();
        double currentPoseRotation = Swerve.swerveOdometry.getEstimatedPosition().getRotation().getDegrees();

        double translationVal = xController.calculate(currentPoseX, targetPoseX);
        double strafeVal = yController.calculate(currentPoseY, targetPoseY);
        double rotationVal = thetaController.calculate(currentPoseRotation, targetPoseRotation);

        if (Math.abs(translationVal) < 0.08) {
            translationVal = 0.0;
        }

        if (Math.abs(strafeVal) < 0.08) {
            strafeVal = 0.0;
        }

        if (Math.abs(rotationVal) < 0.08) {
            rotationVal = 0.0;
        }

        RobotMap.swerve.drive(
            new Translation2d(translationVal, strafeVal).times(1.0), // Constants.SWERVE.MAX_SPEED), 
            rotationVal * Constants.SWERVE.MAX_ANGULAR_VELOCITY * 0.3,
            false, 
            true
        );
    }

    @Override
    public void exit() {
        RobotMap.swerve.drive(
            new Translation2d(0, 0).times(Constants.SWERVE.MAX_SPEED), 
            0 * Constants.SWERVE.MAX_ANGULAR_VELOCITY, 
            true, 
            true
        );
    }

}