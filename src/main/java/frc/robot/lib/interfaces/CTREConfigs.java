package frc.robot.lib.interfaces;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.SHOULDER_MOTOR;
import frc.robot.Constants.ELBOW_MOTOR;
import frc.robot.Constants.SWERVE;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.CLAW;
import frc.robot.Constants.WRIST_MOTOR;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs() {
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SWERVE.ANGLE_ENABLE_CURRENT_LIMIT, 
            Constants.SWERVE.ANGLE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.SWERVE.ANGLE_PEAK_CURRENT_LIMIT, 
            Constants.SWERVE.ANGLE_PEAK_CURRENT_DURATION);

        swerveAngleFXConfig.slot0.kP = Constants.SWERVE.ANGLE_KP;
        swerveAngleFXConfig.slot0.kI = Constants.SWERVE.ANGLE_KI;
        swerveAngleFXConfig.slot0.kD = Constants.SWERVE.ANGLE_KD;
        swerveAngleFXConfig.slot0.kF = Constants.SWERVE.ANGLE_KF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SWERVE.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SWERVE.DRIVE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.SWERVE.DRIVE_PEAK_CURRENT_LIMIT, 
            Constants.SWERVE.DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveFXConfig.slot0.kP = Constants.SWERVE.DRIVE_KP;
        swerveDriveFXConfig.slot0.kI = Constants.SWERVE.DRIVE_KI;
        swerveDriveFXConfig.slot0.kD = Constants.SWERVE.DRIVE_KD;
        swerveDriveFXConfig.slot0.kF = Constants.SWERVE.DRIVE_KF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SWERVE.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = Constants.SWERVE.CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SWERVE.CANCODER_INVERT;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Arm motors PIDF config values */
        RobotMap.shoulderMotor.config_kF(0, SHOULDER_MOTOR.F, 10);
        RobotMap.shoulderMotor.config_kP(0, SHOULDER_MOTOR.P, 10);
        RobotMap.shoulderMotor.config_kI(0, SHOULDER_MOTOR.I, 10);
        RobotMap.shoulderMotor.config_kD(0, SHOULDER_MOTOR.D, 10);

        RobotMap.elbowMotor.config_kF(0, ELBOW_MOTOR.F, 10);
        RobotMap.elbowMotor.config_kP(0, ELBOW_MOTOR.P, 10);
        RobotMap.elbowMotor.config_kI(0, ELBOW_MOTOR.I, 10);
        RobotMap.elbowMotor.config_kD(0, ELBOW_MOTOR.D, 10);

        RobotMap.wristMotor.config_kF(0, WRIST_MOTOR.F, 10);
        RobotMap.wristMotor.config_kP(0, WRIST_MOTOR.P, 10);
        RobotMap.wristMotor.config_kI(0, WRIST_MOTOR.I, 10);
        RobotMap.wristMotor.config_kD(0, WRIST_MOTOR.D, 10);

        /* Claw motors PIDF config values */
        RobotMap.clawMotor.config_kF(0, CLAW.F, 10);
        RobotMap.clawMotor.config_kP(0, CLAW.P, 10);
        RobotMap.clawMotor.config_kI(0, CLAW.I, 10);
        RobotMap.clawMotor.config_kD(0, CLAW.D, 10);
    }
}