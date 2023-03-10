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
import frc.robot.lib.auto.actions.ConditionAction;
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.TrajectoryAction;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.claw.ClawStateMachine;

/** 3 piece auto on the top side of grids */
public class BotSideLoadingStation extends AutoModeBase {
    
    // trajectory action
    TrajectoryAction driveToFirstPiece;
    TrajectoryAction driveToScoreFirstPiece;
    TrajectoryAction driveToLoadingStation;

    public BotSideLoadingStation() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
        
        // transform trajectory depending on alliance we are on
        ArrayList<PathPlannerTrajectory> botSideLoadingStation = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
            "Bot Side Loading Station", 
            new PathConstraints(0.25, 0.25)
        );
        for(int i = 0; i < botSideLoadingStation.size(); i++) {
            botSideLoadingStation.set(
                i, 
                PathPlannerTrajectory.transformTrajectoryForAlliance(botSideLoadingStation.get(i), DriverStation.getAlliance())
            );
        }
        
        driveToFirstPiece = new TrajectoryAction(
            botSideLoadingStation.get(0), 
            RobotMap.swerve::getPose, 
            // () -> Rotation2d.fromDegrees(0.0),
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        driveToScoreFirstPiece = new TrajectoryAction(
            botSideLoadingStation.get(1), 
            RobotMap.swerve::getPose, 
            // () -> Rotation2d.fromDegrees(0.0),
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        driveToLoadingStation = new TrajectoryAction(
            botSideLoadingStation.get(2), 
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

        System.out.println("Running Bot Side Loading Station auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // position arm to score high
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // wait for arm to arrive in position
        runAction(new ConditionAction(() -> {
            return RobotMap.arm.getArrived(Constants.HIGH_SCORE_CONE.ALLOWANCE, Constants.HIGH_SCORE_CONE.TIME);
        }));

        // then, score the piece
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoringState)));

        // wait for the piece to be scored which means the arm is in idle
        runAction(new ConditionAction(() -> {
            return RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.idleState;
        }));

        // drive out of the community to get cube
        runAction(driveToFirstPiece);

        // position arm to pick up
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.groundPickupState)));

        // wait for arm to arrive in position
        runAction(new ConditionAction(() -> {
            return RobotMap.arm.getArrived(Constants.GROUND_INTAKE.ALLOWANCE, Constants.GROUND_INTAKE.TIME);
        }));

        // then, close on the cube
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ClawStateMachine.closingCube)));

        // // wait for the claw to grab onto the cube
        // runAction(new ConditionAction(() -> {
        //     return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedCube;
        // }));

        // position arm to idle
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.idleState)));

        //drive towards grid to score piece
        runAction(driveToScoreFirstPiece);

        // position arm to score high
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // wait for arm to arrive in position
        runAction(new ConditionAction(() -> {
            return RobotMap.arm.getArrived(Constants.HIGH_SCORE_CUBE.ALLOWANCE, Constants.HIGH_SCORE_CUBE.TIME);
        }));

        // then, score the piece
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoringState)));

        // wait for the piece to be scored which means the arm is in idle
        runAction(new ConditionAction(() -> {
            return RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.idleState;
        }));

        // position arm to score high
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // wait for arm to arrive in position
        runAction(new ConditionAction(() -> {
            return RobotMap.arm.getArrived(Constants.HIGH_SCORE_CONE.ALLOWANCE, Constants.HIGH_SCORE_CONE.TIME);
        }));

        // then, score the piece
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoringState)));

        // wait for the piece to be scored which means the arm is in idle
        runAction(new ConditionAction(() -> {
            return RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.idleState;
        }));

        //drive outside community to face loading station
        runAction(driveToLoadingStation);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return driveToFirstPiece.getInitialPose();
    }
}
