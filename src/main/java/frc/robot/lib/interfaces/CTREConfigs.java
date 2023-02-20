package frc.robot.lib.interfaces;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.ShoulderMotorConstants;
import frc.robot.Constants.ElbowMotorConstants;
import frc.robot.Constants.SWERVE;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Constants.Claw;
import frc.robot.Constants.WristMotorConstants;

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

        RobotMap.shoulderMotor.config_kF(0, ShoulderMotorConstants.F, 10);
        RobotMap.shoulderMotor.config_kP(0, ShoulderMotorConstants.P, 10);
        RobotMap.shoulderMotor.config_kI(0, ShoulderMotorConstants.I, 10);
        RobotMap.shoulderMotor.config_kD(0, ShoulderMotorConstants.D, 10);

        RobotMap.elbowMotor.config_kF(0, ElbowMotorConstants.F, 10);
        RobotMap.elbowMotor.config_kP(0, ElbowMotorConstants.P, 10);
        RobotMap.elbowMotor.config_kI(0, ElbowMotorConstants.I, 10);
        RobotMap.elbowMotor.config_kD(0, ElbowMotorConstants.D, 10);

        RobotMap.wristMotor.config_kF(0, WristMotorConstants.F, 10);
        RobotMap.wristMotor.config_kP(0, WristMotorConstants.P, 10);
        RobotMap.wristMotor.config_kI(0, WristMotorConstants.I, 10);
        RobotMap.wristMotor.config_kD(0, WristMotorConstants.D, 10);

        RobotMap.clawMotor.config_kF(0, Claw.F, 10);
        RobotMap.clawMotor.config_kP(0, Claw.P, 10);
        RobotMap.clawMotor.config_kI(0, Claw.I, 10);
        RobotMap.clawMotor.config_kD(0, Claw.D, 10);
    }
}