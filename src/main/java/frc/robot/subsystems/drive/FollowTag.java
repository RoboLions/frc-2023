// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;

/** State to auto-align to a known position on the field */
public class FollowTag extends State {

    private final PIDController xController =
        new PIDController(
            3, 0.01, 0.0
    );

    private final PIDController yController =
        new PIDController(
            3, 0.01, 0.0
    );

    private final PIDController thetaController =
        new PIDController(
            0.1, 0.0, 0.0
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
        thetaController.enableContinuousInput(-180.0, 180.0);
        Pose2d targetPose = RobotMap.swerve.getClosestPose();
        if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) > 1.0) {
            System.out.println("No Close Target!");
            RobotMap.drivetrainStateMachine.setCurrentState(DrivetrainStateMachine.teleopSwerve);
        }
    }

    @Override
    public void execute() {
        if (Swerve.leftShift) {
            RobotMap.swerve.shiftPoseLeft();
        } else if (Swerve.rightShift) {
            RobotMap.swerve.shiftPoseRight();
        }

        Pose2d targetPose = RobotMap.swerve.getClosestPose();
        FieldObject2d targetPoseDisplay = RobotMap.Field2d.getObject("targetPose");
        targetPoseDisplay.setPose(targetPose);
        double targetPoseX = targetPose.getX();
        double targetPoseY = targetPose.getY();
        double targetPoseRotation = targetPose.getRotation().getDegrees();
        // double targetPoseX = 1.034;
        // double targetPoseY = 2.75;
        // double targetPoseRotation = 180.0;

        double currentPoseX = Swerve.swerveOdometry.getEstimatedPosition().getX();
        double currentPoseY = Swerve.swerveOdometry.getEstimatedPosition().getY();
        double currentPoseRotation = Swerve.swerveOdometry.getEstimatedPosition().getRotation().getDegrees();

        double translationVal = xController.calculate(currentPoseX, targetPoseX);
        double strafeVal = yController.calculate(currentPoseY, targetPoseY);
        double rotationVal = thetaController.calculate(currentPoseRotation, targetPoseRotation);

        // SmartDashboard.putNumber("pose x error", targetPoseX - currentPoseX);
        // SmartDashboard.putNumber("pose y error", targetPoseY - currentPoseY);
        // SmartDashboard.putNumber("pose rotation error", targetPoseRotation - currentPoseRotation);

        // System.out.println(targetPoseX + ", " + currentPoseX + ", " + targetPoseY + ", " + currentPoseY + ", " + targetPoseRotation + ", " + currentPoseRotation);

        // if (Math.abs(translationVal) < 0.08) {
        //     translationVal = 0.0;
        // }

        // if (Math.abs(strafeVal) < 0.08) {
        //     strafeVal = 0.0;
        // }

        // if (Math.abs(rotationVal) < 0.08) {
        //     rotationVal = 0.0;
        // }

        translationVal = Math.min(Math.max(translationVal, -0.5), 0.5);
        strafeVal = Math.min(Math.max(strafeVal, -0.7), 0.7);
        rotationVal = Math.min(Math.max(rotationVal, -2.0), 2.0);

        translationVal *= DriverStation.getAlliance() == DriverStation.Alliance.Blue ? -1.0 : 1.0;
        strafeVal *= DriverStation.getAlliance() == DriverStation.Alliance.Blue ? -1.0 : 1.0;

        RobotMap.swerve.drive(
            new Translation2d(translationVal, strafeVal), // Constants.SWERVE.MAX_SPEED), 
            rotationVal,
            true, 
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