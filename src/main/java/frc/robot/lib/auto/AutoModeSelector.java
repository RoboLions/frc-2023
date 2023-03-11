// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.autos.BotSideLink;
import frc.robot.subsystems.drive.autos.BotSideLoadingStation;
import frc.robot.subsystems.drive.autos.BotSimpleScore;
import frc.robot.subsystems.drive.autos.DoNothing;
import frc.robot.subsystems.drive.autos.MidScoreBalance;
import frc.robot.subsystems.drive.autos.TestDrivePath;
import frc.robot.subsystems.drive.autos.TopSideLink;
import frc.robot.subsystems.drive.autos.TopSideLoadingStation;
import frc.robot.subsystems.drive.autos.TopSimpleScore;

public class AutoModeSelector {
    enum DesiredMode {
        DO_NOTHING, 
        TEST_PATH,
        BOT_SIMPLE_SCORE,
        TOP_SIMPLE_SCORE,
        MID_SCORE_BALANCE,
        BOT_LOADING_STATION,
        TOP_LOADING_STATION,
        BOT_LINK,
        TOP_LINK
    }

    private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

    private Optional<AutoModeBase> mAutoMode = Optional.empty();

    private SendableChooser<DesiredMode> mModeChooser;

    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.setDefaultOption("Do Nothing", DesiredMode.DO_NOTHING);
        mModeChooser.addOption("Test", DesiredMode.TEST_PATH);
        mModeChooser.addOption("Bot Simple Score", DesiredMode.BOT_SIMPLE_SCORE);
        mModeChooser.addOption("Top Simple Score", DesiredMode.TOP_SIMPLE_SCORE);
        mModeChooser.addOption("Mid Balance", DesiredMode.MID_SCORE_BALANCE);
        mModeChooser.addOption("Bot Loading Station", DesiredMode.BOT_LOADING_STATION);
        mModeChooser.addOption("Top Loading Station", DesiredMode.TOP_LOADING_STATION);
        mModeChooser.addOption("Bot Link", DesiredMode.BOT_LINK);
        mModeChooser.addOption("Top Link", DesiredMode.TOP_LINK);
        
        SmartDashboard.putData("Auto Mode", mModeChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (desiredMode == null) {
            desiredMode = DesiredMode.DO_NOTHING;
        }
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            mAutoMode = getAutoModeForParams(desiredMode);
        }
        mCachedDesiredMode = desiredMode;
    }

    private Optional<AutoModeBase> getAutoModeForParams(DesiredMode mode) {
        switch (mode) {
        case DO_NOTHING:
            return Optional.of(new DoNothing());

        case TEST_PATH:
            return Optional.of(new TestDrivePath());

        case BOT_SIMPLE_SCORE:
            return Optional.of(new BotSimpleScore());

        case TOP_SIMPLE_SCORE:
            return Optional.of(new TopSimpleScore());

        case MID_SCORE_BALANCE:
            return Optional.of(new MidScoreBalance());

        case BOT_LOADING_STATION:
            return Optional.of(new BotSideLoadingStation());

        case TOP_LOADING_STATION:
            return Optional.of(new TopSideLoadingStation());

        case BOT_LINK:
            return Optional.of(new BotSideLink());
        
        case TOP_LINK:
            return Optional.of(new TopSideLink());
            
        default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
            break;
        }

        System.err.println("No valid auto mode found for  " + mode);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

    public Optional<AutoModeBase> getAutoMode() {
        if (!mAutoMode.isPresent()) {
            return Optional.empty();
        }
        return mAutoMode;
    }

    public boolean isDriveByCamera() {
        return mCachedDesiredMode == DesiredMode.DO_NOTHING;
    }
}
