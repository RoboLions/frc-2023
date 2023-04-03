// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
import frc.robot.subsystems.intake.IntakeStateMachine;

/** Simple auto path for top side of grids, 1 cone high */
public class TopSimpleScore extends AutoModeBase {
    
    // trajectory action
    TrajectoryAction driveOut;

    Pose2d initialHolonomicPose;

    private Timer timer = new Timer();
    
    public TopSimpleScore() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
        
        // transform trajectory depending on alliance we are on
        PathPlannerTrajectory topSimpleScore = PathPlanner.loadPath("Top Simple Score", new PathConstraints(1.5, 0.5));
        topSimpleScore = PathPlannerTrajectory.transformTrajectoryForAlliance(topSimpleScore, DriverStation.getAlliance());
        
        initialHolonomicPose = topSimpleScore.getInitialHolonomicPose();

        driveOut = new TrajectoryAction(
            topSimpleScore, 
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

        System.out.println("Running top simple score auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // position arm to score high
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // wait for arm to arrive in position
        runAction(new ConditionAction(() -> {
            return Arm.getArrived(Constants.HIGH_SCORE_CONE.SHOULDER_ALLOWANCE, Constants.HIGH_SCORE_CONE.ELBOW_ALLOWANCE, Constants.HIGH_SCORE_CONE.TIME);
        }));

        // then, score the piece
        timer.start();
        runAction(new LambdaAction(() -> RobotMap.intakeStateMachine.maintainState(IntakeStateMachine.outtakingState)));

        // wait for the piece to be scored
        runAction(new ConditionAction(() -> {
            return timer.hasElapsed(Constants.INTAKE.OUTTAKE_TIME);
        }));

        runAction(new LambdaAction(() -> RobotMap.intakeStateMachine.setCurrentState(IntakeStateMachine.idleState)));

        // put arm to idle
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.elbowIdleState)));

        // drive out of the community
        runAction(driveOut);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return initialHolonomicPose;
    }
}
