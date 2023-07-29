package frc.robot.lib.interfaces;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.Constants;

import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.CTREModuleState;
import frc.robot.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModuleFalcon500 implements SwerveModuleIO{

    private Rotation2d lastAngle;

    public TalonFX mAngleMotor;
    public TalonFX mDriveMotor;
    private CANCoder angleEncoder;

    private static CTREConfigs ctreConfigs;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SWERVE.DRIVE_KS, Constants.SWERVE.DRIVE_KV, Constants.SWERVE.DRIVE_KA);



    public SwerveModuleFalcon500(SwerveModuleConstants moduleConstants){

        ctreConfigs = new CTREConfigs();
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
               
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, SwerveModuleState state){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, state.angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SWERVE.MAX_SPEED;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SWERVE.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
        mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.SWERVE.ANGLE_GEAR_RATIO));
        lastAngle = angle;
    }

    // private Rotation2d getAngle(){
    //     return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.SWERVE.ANGLE_GEAR_RATIO));
    // }

    // public Rotation2d getCanCoder(){
    //     return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    // }

    public void resetToAbsolute(double absolutePosition){
        // double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.SWERVE.ANGLE_GEAR_RATIO);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.SWERVE.ANGLE_MOTOR_INVERT);
        mAngleMotor.setNeutralMode(Constants.SWERVE.ANGLE_NEUTRAL_MODE);
        // resetToAbsolute(absolutePosition);
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.SWERVE.DRIVE_MOTOR_INVERT);
        mDriveMotor.setNeutralMode(Constants.SWERVE.DRIVE_NEUTRAL_MODE);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    // public SwerveModuleState getState(){
    //     return new SwerveModuleState(
    //         Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO), 
    //         getAngle()
    //     ); 
    // }

    // public SwerveModulePosition getPosition(){
    //     return new SwerveModulePosition(
    //         Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO), 
    //         getAngle()
    //     );
    // }

    public void updateInputs(SwerveModuleIOInputs inputs) {

        inputs.driveMotorSensorPos = mDriveMotor.getSelectedSensorPosition();
        inputs.driveMotorSensorVelocity = mDriveMotor.getSelectedSensorVelocity();
       
        inputs.driveOutputVoltageVolts = mDriveMotor.getMotorOutputPercent() * mDriveMotor.getBusVoltage(); // Might be Incorrect


        inputs.driveTempCelcius = new double[] {mDriveMotor.getTemperature()};
    
        inputs.angleEncoderAbsolutePos = angleEncoder.getAbsolutePosition();

        inputs.angleMotorSensorPosition = mAngleMotor.getSelectedSensorPosition();
        inputs.turnVelocityMetersPerSec =Conversions.falconToMeters(mDriveMotor.getSelectedSensorVelocity(), Constants.SWERVE.WHEEL_CIRCUMFERENCE, Constants.SWERVE.DRIVE_GEAR_RATIO);

        inputs.turnAppliedVolts = mAngleMotor.getMotorOutputPercent() * mAngleMotor.getBusVoltage(); // Might be Incorrect

        inputs.turnTempCelcius = new double[] {mAngleMotor.getTemperature()};

}
}