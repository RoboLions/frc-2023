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
            Constants.SWERVE.angleEnableCurrentLimit, 
            Constants.SWERVE.angleContinuousCurrentLimit, 
            Constants.SWERVE.anglePeakCurrentLimit, 
            Constants.SWERVE.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = Constants.SWERVE.angleKP;
        swerveAngleFXConfig.slot0.kI = Constants.SWERVE.angleKI;
        swerveAngleFXConfig.slot0.kD = Constants.SWERVE.angleKD;
        swerveAngleFXConfig.slot0.kF = Constants.SWERVE.angleKF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SWERVE.driveEnableCurrentLimit, 
            Constants.SWERVE.driveContinuousCurrentLimit, 
            Constants.SWERVE.drivePeakCurrentLimit, 
            Constants.SWERVE.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = Constants.SWERVE.driveKP;
        swerveDriveFXConfig.slot0.kI = Constants.SWERVE.driveKI;
        swerveDriveFXConfig.slot0.kD = Constants.SWERVE.driveKD;
        swerveDriveFXConfig.slot0.kF = Constants.SWERVE.driveKF;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SWERVE.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = Constants.SWERVE.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SWERVE.canCoderInvert;
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