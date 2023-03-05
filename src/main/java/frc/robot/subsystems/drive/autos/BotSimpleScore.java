// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.autos;

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
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.TrajectoryAction;
import frc.robot.subsystems.arm.ArmStateMachine;

/** Simple bot score, 1 cone high */
public class BotSimpleScore extends AutoModeBase {
    
    // trajectory action
    TrajectoryAction driveOut;

    public BotSimpleScore() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
        
        // transform trajectory depending on alliance we are on
        PathPlannerTrajectory botSimpleScore = PathPlanner.loadPath("Bot Simple Score", new PathConstraints(0.25, 0.25));
        SmartDashboard.putNumber("rotation pose original", botSimpleScore.getInitialHolonomicPose().getRotation().getDegrees());
        // List<State> states_before = botSimpleScore.getStates();
        // botSimpleScore = PathPlannerTrajectory.transformTrajectoryForAlliance(botSimpleScore, DriverStation.getAlliance());
        // List<State> states_after = botSimpleScore.getStates();
        // boolean temp = false;
        // for (int i = 0; i < states_before.size(); i++) {
        //     if (states_before.get(i).equals(states_after.get(i))) {
        //         temp = true;
        //     }
        // }
        // if (temp) {
        //     System.out.println("SOME STATES WERE THE SAME");
        // }
        // else {
        //     System.out.println("ALL STATES WERE DIFFERENT");
        // }

        driveOut = new TrajectoryAction(
            botSimpleScore, 
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

        System.out.println("Running bot simple score auto!");
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

        // drive out of the community
        runAction(driveOut);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        double xPose = driveOut.getInitialPose().getX();
        double yPose =  driveOut.getInitialPose().getY();
        double rotation =  driveOut.getInitialPose().getRotation().getDegrees();
        SmartDashboard.putNumber("X pose", xPose);
        SmartDashboard.putNumber("Y pose", yPose);
        SmartDashboard.putNumber("rotation pose", rotation);
        return driveOut.getInitialPose();
    }
}
