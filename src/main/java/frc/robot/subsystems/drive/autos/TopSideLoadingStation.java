// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.autos;

import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeEndedException;
import frc.robot.lib.auto.actions.ConditionAction;
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.TrajectoryAction;
import frc.robot.lib.interfaces.Arm;
import frc.robot.subsystems.arm.ArmStateMachine;

/** 3 piece auto on the top side of grids */
public class TopSideLoadingStation extends AutoModeBase {
    
    // trajectory action
    TrajectoryAction driveToFirstPiece;
    TrajectoryAction driveToScoreFirstPiece;
    TrajectoryAction driveToLoadingStation;

    Pose2d initialHolonomicPose;

    public TopSideLoadingStation() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
        thetaController.enableContinuousInput(-180.0, 180.0);
        
        // transform trajectory depending on alliance we are on
        ArrayList<PathPlannerTrajectory> topSideLoadingStation = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
            "Top Side Loading Station", 
            new PathConstraints(2.5, 1.0)
        );

        for(int i = 0; i < topSideLoadingStation.size(); i++) {
            topSideLoadingStation.set(
                i, 
                PathPlannerTrajectory.transformTrajectoryForAlliance(topSideLoadingStation.get(i), DriverStation.getAlliance())
            );
        }
        
        initialHolonomicPose = topSideLoadingStation.get(0).getInitialHolonomicPose();
        
        driveToFirstPiece = new TrajectoryAction(
            topSideLoadingStation.get(0), 
            RobotMap.swerve::getPose, 
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        driveToScoreFirstPiece = new TrajectoryAction(
            topSideLoadingStation.get(1), 
            RobotMap.swerve::getPose, 
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        driveToLoadingStation = new TrajectoryAction(
            topSideLoadingStation.get(2), 
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

        System.out.println("Running Top Side Loading Station auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // close the claw
        // runAction(new LambdaAction(() -> RobotMap.clawStateMachine.maintainState(ClawStateMachine.closingState)));
        
        // // position arm to score high
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // // wait for arm to arrive in position
        // runAction(new ConditionAction(() -> {
        //     return Arm.getArrived(Constants.HIGH_SCORE_CONE.ALLOWANCE, Constants.HIGH_SCORE_CONE.TIME);
        // }));

        // // then, score the piece
        // runAction(new LambdaAction(() -> RobotMap.clawStateMachine.setCurrentState(ClawStateMachine.closedState)));
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoringState)));

        // wait for the piece to be scored which means the arm is in idle
        runAction(new ConditionAction(() -> {
            return RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.elbowIdleState;
        }));

        // position arm to pick up
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.groundPickupState)));

        // drive out of the community to get cube
        runAction(driveToFirstPiece);

        // wait for arm to arrive in position
        // runAction(new ConditionAction(() -> {
        //     return Arm.getArrived(Constants.GROUND_INTAKE.ALLOWANCE, Constants.GROUND_INTAKE.TIME);
        // }));

        // // then, close on the cube
        // runAction(new LambdaAction(() -> Claw.requestClawClosed()));

        // // wait for the claw to grab onto the cube
        // runAction(new ConditionAction(() -> {
        //     return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedState;
        // }));

        // //drive towards grid to score piece
        // runAction(driveToScoreFirstPiece);

        // // position arm to score high
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // // wait for arm to arrive in position
        // runAction(new ConditionAction(() -> {
        //     return RobotMap.arm.getArrived(Constants.HIGH_SCORE_CUBE.ALLOWANCE, Constants.HIGH_SCORE_CUBE.TIME);
        // }));

        // // then, score the piece
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoringState)));

        // // wait for the piece to be scored which means the arm is in idle
        // runAction(new ConditionAction(() -> {
        //     return RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.idleState;
        // }));

        // //drive outside community to face loading station
        // runAction(driveToLoadingStation);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return initialHolonomicPose;
    }
}
