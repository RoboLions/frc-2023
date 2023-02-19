// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeEndedException;
import frc.robot.lib.auto.actions.TrajectoryAction;

/** Simple auto path for testing */
public class TestPath extends AutoModeBase {

    // This will load the file "Test Path.path" and generate it with a max velocity of 0.2 m/s and a max acceleration of 0.2 m/s^2
    static PathPlannerTrajectory testPath = PathPlanner.loadPath("Test Path", new PathConstraints(0.2, 0.2));

    // trajectory action
    TrajectoryAction testDrive;

    public TestPath() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = Constants.Swerve.Profile.THETA_CONTROLLER;
    
        testDrive = new TrajectoryAction(
            testPath, 
            RobotMap.swerve::getPose, 
            Constants.Swerve.swerveKinematics, 
            Constants.Swerve.Profile.X_CONTROLLER,
            Constants.Swerve.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        System.out.println("Running bot simple score auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // drive
        runAction(testDrive);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return testDrive.getInitialPose();
    }
}
