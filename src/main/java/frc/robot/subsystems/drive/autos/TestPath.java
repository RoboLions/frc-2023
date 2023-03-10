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
public class TestPath extends AutoModeBase {

    // trajectory action
    TrajectoryAction testDrive1;
    TrajectoryAction testDrive2;

    public TestPath() {
        
        SmartDashboard.putBoolean("Auto Finished", false);
        
        ArrayList<PathPlannerTrajectory> testPath = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
            "Test Path", 
            new PathConstraints(0.25, 0.25)
        );

        for(int i = 0; i < testPath.size(); i++) {
            testPath.set(
                i, 
                PathPlannerTrajectory.transformTrajectoryForAlliance(testPath.get(i), DriverStation.getAlliance())
            );
        }

        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
    
        testDrive1 = new TrajectoryAction(
            testPath.get(0), 
            RobotMap.swerve::getPose, 
            // () -> Rotation2d.fromDegrees(0.0),
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        testDrive2 = new TrajectoryAction(
            testPath.get(1), 
            RobotMap.swerve::getPose, 
            // () -> Rotation2d.fromDegrees(0.0),
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

        runAction(testDrive2);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return testDrive1.getInitialPose();
    }
}
