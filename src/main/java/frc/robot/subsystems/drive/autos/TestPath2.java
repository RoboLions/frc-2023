// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.autos;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeEndedException;
import frc.robot.lib.auto.actions.EmptyAction;
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.TrajectoryAction;
import frc.robot.lib.auto.actions.WaitAction;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Simple auto path for testing */
public class TestPath2 extends AutoModeBase {

    static PathPlannerTrajectory testPath = PathPlanner.loadPath("Test Path", new PathConstraints(1.50, 0.50));
    // trajectory action
    TrajectoryAction testDrive1;

    public TestPath2() {
        
        SmartDashboard.putBoolean("Auto Finished", false);
        
        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
    
        testDrive1 = new TrajectoryAction(
            testPath, 
            RobotMap.swerve::getPose, 
            // () -> Rotation2d.fromDegrees(180.0),
            // () -> testPath.sample(testPath.getTotalTimeSeconds()).poseMeters.getRotation(),
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        System.out.println("Running test auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // drive
        runAction(testDrive1);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return testPath.getInitialHolonomicPose();
    }
}
