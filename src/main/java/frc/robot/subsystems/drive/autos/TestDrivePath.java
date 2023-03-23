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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeEndedException;
import frc.robot.lib.auto.actions.ConditionAction;
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.TrajectoryAction;
import frc.robot.lib.auto.actions.WaitAction;
import frc.robot.lib.interfaces.Arm;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.subsystems.arm.ArmStateMachine;
import frc.robot.subsystems.claw.ClawStateMachine;

/** Simple auto path for testing */
public class TestDrivePath extends AutoModeBase {

    Pose2d initialHolonomicPose;

    // trajectory action
    TrajectoryAction testDrive;

    public TestDrivePath() {
        
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

        initialHolonomicPose = testPath.get(0).getInitialHolonomicPose();
        
        // define theta controller for robot heading
        var thetaController = Constants.SWERVE.Profile.THETA_CONTROLLER;
    
        testDrive = new TrajectoryAction(
            testPath.get(0), 
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

        System.out.println("Running test auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // close the claw
        // runAction(new LambdaAction(() -> RobotMap.clawStateMachine.maintainState(ClawStateMachine.closingState)));

        // // wait for claw to be in closed state
        // // runAction(new ConditionAction(() -> {
        // //     return RobotMap.clawStateMachine.getCurrentState() == ClawStateMachine.closedState;
        // // }));
        
        // // position arm to score high
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoreHighState)));

        // // wait for arm to arrive in position
        // runAction(new ConditionAction(() -> {
        //     return Arm.getArrived(Constants.HIGH_SCORE_CONE.ALLOWANCE, Constants.HIGH_SCORE_CONE.TIME);
        // }));

        // // then, score the piece
        // runAction(new LambdaAction(() -> RobotMap.clawStateMachine.setCurrentState(ClawStateMachine.closedState)));
        // runAction(new LambdaAction(() -> RobotMap.armStateMachine.setCurrentState(ArmStateMachine.scoringState)));

        // // wait for the piece to be scored which means the arm is in idle
        // runAction(new ConditionAction(() -> {
        //     return RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.idleState;
        // }));
        
        // drive
        runAction(testDrive);

        //runAction(new WaitAction(2.0));

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return initialHolonomicPose;
    }
}
