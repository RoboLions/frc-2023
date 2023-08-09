// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces.Swerve;

import org.littletonrobotics.junction.AutoLog;


import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
      public double driveMotorSensorPos = 0.0;
      public double driveMotorSensorVelocity = 0.0;
      public double driveOutputVoltageVolts = 0.0;

      public double[] driveTempCelcius = new double[] {};
  
      public double angleEncoderAbsolutePos = 0.0;
      public double absolutePosition = 0.0;


      public double angleMotorSensorPosition = 0.0;
      public double turnVelocityMetersPerSec = 0.0;
      public double turnAppliedVolts = 0.0;

      public double[] turnTempCelcius = new double[] {};
    }
  
    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveModuleIOInputs inputs) {}
    
    public default void resetToAbsolute(double absolutePosition){}

    public default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, SwerveModuleState state){}

    public default void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){}
    
    public default void setAngle(SwerveModuleState desiredState){}
  }
