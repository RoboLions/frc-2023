// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.lib.auto.actions.Action;
import frc.robot.lib.auto.actions.EmptyAction;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.subsystems.drive.autos.TopTwoPieceRed;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
    protected final double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;
    protected boolean mIsInterrupted = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        mActive = true;

        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done() {
        if (AutoModeSelector.mCachedDesiredMode.name() != "TOP_TWO_PIECE_RED" && AutoModeSelector.mCachedDesiredMode.name() != "TEST_PATH") {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                Pose2d currentPose = Swerve.swerveOdometry.getEstimatedPosition();
                // System.out.println(currentPose);
                Pose2d newPose = currentPose.transformBy(new Transform2d(new Translation2d(0, 0), Rotation2d.fromDegrees(180.0)));
                RobotMap.swerve.resetOdometry(newPose);
                // Pose2d finalPose = Swerve.swerveOdometry.getEstimatedPosition();
                // System.out.println(finalPose);
                // Pose2d newPose = Swerve.swerveOdometry.getEstimatedPosition();
                // System.out.println(newPose);
            }
        }
        System.out.println("Auto mode done");
    }

    public void stop() {
        mActive = false;
    }

    public boolean isActive() {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void waitForDriverConfirm() throws AutoModeEndedException {
        if (!mIsInterrupted) {
            interrupt();
        }
        runAction(new EmptyAction());
    }

    public void interrupt() {
        System.out.println("** Auto mode interrrupted!");
        mIsInterrupted = true;
    }

    public void resume() {
        System.out.println("** Auto mode resumed!");
        mIsInterrupted = false;
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        long waitTime = (long) (mUpdateRate * 1000.0);

        // Wait for interrupt state to clear
        while (isActiveWithThrow() && mIsInterrupted) {
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.start();

        // Run action, stop action on interrupt, non active mode, or done
        while (isActiveWithThrow() && !action.isFinished() && !mIsInterrupted) {
            action.update();

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();

    }

    public boolean getIsInterrupted() {
        return mIsInterrupted;
    }

    public abstract Pose2d getStartingPose();
}
