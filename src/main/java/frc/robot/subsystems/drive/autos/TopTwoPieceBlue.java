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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeEndedException;
import frc.robot.lib.auto.actions.ConditionAction;
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.ParallelAction;
import frc.robot.lib.auto.actions.SeriesAction;
import frc.robot.lib.auto.actions.TrajectoryAction;
import frc.robot.lib.interfaces.Arm;
import frc.robot.subsystems.LED.LEDStateMachine;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.intake.IntakeStateMachine;

/** 3 piece auto on the top side of grids */
public class TopTwoPieceBlue extends AutoModeBase {
    
    // trajectory action
    TrajectoryAction driveToIntake;
    TrajectoryAction driveToScore;

    private Timer timer = new Timer();
    private Timer timer2 = new Timer();
    private Timer timer3 = new Timer();
    
    Pose2d initialHolonomicPose;

    public TopTwoPieceBlue() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
        thetaController.enableContinuousInput(-180.0, 180.0);
        
        ArrayList<PathPlannerTrajectory> topTwoPiece = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
            "Top Side Two Piece Blue", 
            new PathConstraints(3.0, 2.0)
        );

        initialHolonomicPose = topTwoPiece.get(0).getInitialHolonomicPose();

        driveToIntake = new TrajectoryAction(
            topTwoPiece.get(0), 
            RobotMap.swerve::getPose, 
            Constants.SWERVE.SWERVE_KINEMATICS, 
            Constants.SWERVE.Profile.X_CONTROLLER,
            Constants.SWERVE.Profile.Y_CONTROLLER,
            thetaController,
            RobotMap.swerve::setModuleStates
        );

        driveToScore = new TrajectoryAction(
            topTwoPiece.get(1),
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

        // position arm to score high
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // wait for arm to arrive in position
        runAction(new ConditionAction(() -> {
            return Arm.getArrived(Constants.HIGH_SCORE_CONE.ALLOWANCE, Constants.HIGH_SCORE_CONE.TIME);
        }));

        // then, score the piece
        timer.start();
        runAction(new LambdaAction(() -> RobotMap.intakeStateMachine.maintainState(IntakeStateMachine.outtakingState)));

        // wait for the piece to be scored
        runAction(new ConditionAction(() -> {
            return timer.hasElapsed(Constants.INTAKE.OUTTAKE_TIME);
        }));

        // stop intake
        runAction(new LambdaAction(() -> RobotMap.intakeStateMachine.setCurrentState(IntakeStateMachine.idleState)));

        // put arm to idle
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.maintainState(ArmStateMachine.elbowIdleState)));

        runAction(new ConditionAction(() -> {
            return Arm.getArrived(Constants.ELBOW_IDLE.ALLOWANCE, Constants.ELBOW_IDLE.TIME);
        }));

        // put arm to idle
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.idleState)));

        // put LED to cube (purple)
        runAction(new LambdaAction(() -> RobotMap.ledStateMachine.setCurrentState(LEDStateMachine.cubeLEDState)));

        // put arm to ground intake position
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.groundPickupState)));

        // run intake
        runAction(new LambdaAction(() -> RobotMap.intakeStateMachine.maintainState(IntakeStateMachine.intakingState)));

        // drive out of the community to get cube
        runAction(driveToIntake);

        // stop intake
        runAction(new LambdaAction(() -> RobotMap.intakeStateMachine.setCurrentState(IntakeStateMachine.idleState)));

        // put arm to elbow idle
        runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.elbowIdleState)));

        timer3.start();

        // position arm to score high after 1 second
        runAction(new ParallelAction(
            List.of(
                driveToScore,
                new SeriesAction(
                    List.of(
                        new ConditionAction(() -> {
                            return timer3.hasElapsed(1.0);
                        }),
                    new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState))))
            ))
        );

        // wait for arm to arrive in position
        runAction(new ConditionAction(() -> {
            return Arm.getArrived(Constants.HIGH_SCORE_CONE.ALLOWANCE, Constants.HIGH_SCORE_CONE.TIME);
        }));

        // then, score the piece
        timer2.start();
        runAction(new LambdaAction(() -> RobotMap.intakeStateMachine.maintainState(IntakeStateMachine.outtakingState)));

        // wait for the piece to be scored
        runAction(new ConditionAction(() -> {
            return timer2.hasElapsed(Constants.INTAKE.OUTTAKE_TIME);
        }));

        // stop intake
        runAction(new LambdaAction(() -> RobotMap.intakeStateMachine.setCurrentState(IntakeStateMachine.idleState)));

        // put arm to idle
       // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.elbowIdleState)));

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return initialHolonomicPose;
    }
}
