// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.autos;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeEndedException;
import frc.robot.lib.auto.actions.ConditionAction;
import frc.robot.lib.auto.actions.EmptyAction;
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.TrajectoryAction;
import frc.robot.lib.auto.actions.WaitAction;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Simple auto path for testing */
public class TopSideLink extends AutoModeBase {
    
    // trajectory action
    TrajectoryAction driveToFirstPiece;
    TrajectoryAction driveToScoreFirstPiece;
    TrajectoryAction driveToSecondPiece;
    TrajectoryAction driveToScoreSecondPiece;

    public TopSideLink() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
        
        // transform trajectory depending on alliance we are on
        ArrayList<PathPlannerTrajectory> topSideLink = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
            "Top Side Link Auto", 
            new PathConstraints(0.25, 0.25)
        );
        for(int i = 0; i < topSideLink.size(); i++) {
            topSideLink.set(
                i, 
                PathPlannerTrajectory.transformTrajectoryForAlliance(topSideLink.get(i), DriverStation.getAlliance())
            );
        }
        

        driveToFirstPiece = new TrajectoryAction(
            topSideLink.get(0), 
            RobotMap.swerve::getPose, 
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        driveToScoreFirstPiece = new TrajectoryAction(
            topSideLink.get(1), 
            RobotMap.swerve::getPose, 
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        driveToSecondPiece = new TrajectoryAction(
            topSideLink.get(2), 
            RobotMap.swerve::getPose, 
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        driveToScoreSecondPiece = new TrajectoryAction(
            topSideLink.get(3), 
            RobotMap.swerve::getPose, 
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

    }

    @Override
    protected void routine() throws AutoModeEndedException {

        System.out.println("Running Top Side Link auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // drive out of the community to get first piece
        runAction(driveToFirstPiece);

        //drive towards grid to score piece
        runAction(driveToScoreFirstPiece);

        //drive outside community to get the second piece
        runAction(driveToSecondPiece);

        //drive towards the grid to score second piece
        runAction(driveToScoreSecondPiece);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return driveToFirstPiece.getInitialPose();
    }
}
