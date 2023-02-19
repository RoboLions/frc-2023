// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.lib.states.Swerve;

/** State to auto-align to a known position on the field */
public class FollowTag extends State {

    HolonomicDriveController m_controller;

    @Override
    public void build() {
        // if driver joysticks are engaged, transition to teleop state
        addTransition(new Transition(() -> {
            return Math.abs(RobotMap.driverController.getLeftX()) > Constants.stickDeadband || 
                Math.abs(RobotMap.driverController.getLeftY()) > Constants.stickDeadband || 
                Math.abs(RobotMap.driverController.getRightX()) > Constants.stickDeadband ||
                Math.abs(RobotMap.driverController.getRightY()) > Constants.stickDeadband;
        }, DrivetrainStateMachine.teleopSwerve));
    }

    @Override
    public void init() {
        Pose2d currentPose = Swerve.swerveOdometry.getEstimatedPosition();
        RobotMap.swerve.setClosestPose(currentPose);
        
        m_controller = new HolonomicDriveController(
            Constants.Swerve.Profile.X_CONTROLLER,
            Constants.Swerve.Profile.Y_CONTROLLER,
            Constants.Swerve.Profile.THETA_CONTROLLER
        );
    }

    @Override
    public void execute() {
        if (RobotMap.driverController.getRawButtonPressed(Constants.DriverButtons.SHIFT_LEFT_BUTTON)) {
            RobotMap.swerve.shiftPoseLeft();
        } else if (RobotMap.driverController.getRawButtonPressed(Constants.DriverButtons.SHIFT_RIGHT_BUTTON)) {
            RobotMap.swerve.shiftPoseRight();
        }
        Pose2d targetPose = RobotMap.swerve.getClosestPose();
        Pose2d currentPose = Swerve.swerveOdometry.getEstimatedPosition();
        ChassisSpeeds targetChassisSpeeds = m_controller.calculate(currentPose, targetPose, 0.25, targetPose.getRotation());
        SwerveModuleState[] outputModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds);

        RobotMap.swerve.setModuleStates(outputModuleStates);
    }

    @Override
    public void exit() {
        RobotMap.swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
        );
    }

}