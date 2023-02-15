// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.auto.AutoModeBase;
import frc.robot.lib.auto.AutoModeEndedException;
import frc.robot.lib.auto.actions.LambdaAction;
import frc.robot.lib.auto.actions.TrajectoryAction;

/** Add your docs here. */
public class BotSimpleScore extends AutoModeBase {

    // This will load the file "Bot Simple Score.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // TODO: check if file path is right
    static PathPlannerTrajectory botSimpleScorePath = PathPlanner.loadPath("Bot Simple Score", new PathConstraints(4, 3));

    // trajectory action
    TrajectoryAction botSimpleScore;

    public BotSimpleScore() {

        SmartDashboard.putBoolean("Auto Finished", false);

        // define theta controller for robot heading
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
        botSimpleScore = new TrajectoryAction(
            botSimpleScorePath, 
            RobotMap.swerve::getPose, 
            Constants.Swerve.swerveKinematics, 
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            RobotMap.swerve::setModuleStates
        );
    }

    @Override
    protected void routine() throws AutoModeEndedException {

        System.out.println("Running bot simple score auto!");
        SmartDashboard.putBoolean("Auto Finished", false);

        // reset odometry at the start of the trajectory
        runAction(new LambdaAction(() -> RobotMap.swerve.resetOdometry(new Pose2d(botSimpleScore.getInitialPose().getX(),
                                                                        botSimpleScore.getInitialPose().getY(),
                                                                        Rotation2d.fromDegrees(0.0)))));

        // drive
        runAction(botSimpleScore);

        System.out.println("Finished auto!");
        SmartDashboard.putBoolean("Auto Finished", true);
    }

    @Override
    public Pose2d getStartingPose() {
        return botSimpleScore.getInitialPose();
    }
}
