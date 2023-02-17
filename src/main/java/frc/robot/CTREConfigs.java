package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.ArmFirstStageConstants;
import frc.robot.Constants.ArmSecondStageConstants;
import frc.robot.Constants.Wrist;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.angleEnableCurrentLimit, 
            Constants.Swerve.angleContinuousCurrentLimit, 
            Constants.Swerve.anglePeakCurrentLimit, 
            Constants.Swerve.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.Swerve.driveEnableCurrentLimit, 
            Constants.Swerve.driveContinuousCurrentLimit, 
            Constants.Swerve.drivePeakCurrentLimit, 
            Constants.Swerve.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        RobotMap.armFirstStageMotor.config_kF(0, ArmFirstStageConstants.F, 10);
        RobotMap.armFirstStageMotor.config_kP(0, ArmFirstStageConstants.P, 10);
        RobotMap.armFirstStageMotor.config_kI(0, ArmFirstStageConstants.I, 10);
        RobotMap.armFirstStageMotor.config_kD(0, ArmFirstStageConstants.D, 10);

        RobotMap.armSecondStageMotor.config_kF(0, ArmSecondStageConstants.F, 10);
        RobotMap.armSecondStageMotor.config_kP(0, ArmSecondStageConstants.P, 10);
        RobotMap.armSecondStageMotor.config_kI(0, ArmSecondStageConstants.I, 10);
        RobotMap.armSecondStageMotor.config_kD(0, ArmSecondStageConstants.D, 10);

        RobotMap.wristMotor.config_kF(0, Wrist.F, 10);
        RobotMap.wristMotor.config_kP(0, Wrist.P, 10);
        RobotMap.wristMotor.config_kI(0, Wrist.I, 10);
        RobotMap.wristMotor.config_kD(0, Wrist.D, 10);
    }
}