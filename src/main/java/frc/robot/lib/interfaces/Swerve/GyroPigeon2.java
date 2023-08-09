// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.interfaces.Swerve;


import com.ctre.phoenix.sensors.WPI_Pigeon2;


/** Add your docs here. */
public class GyroPigeon2 implements GyroIO{

    private static WPI_Pigeon2 gyro;

    
    public GyroPigeon2(int id){
        gyro = new WPI_Pigeon2(id);
        gyro.configFactoryDefault();
    }
    
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();
      }

      public void zeroGyro(){
        gyro.setYaw(0);
    }
}
