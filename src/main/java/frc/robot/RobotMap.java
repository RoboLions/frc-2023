package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;

import frc.robot.lib.Swerve;

public class RobotMap {
    
    /* Motor IDs */
    public static final int pigeonID = 5;
    
    /* Motor instances */
    public static Pigeon2 gyro = new Pigeon2(pigeonID);

    /* Swerve modules instance */
    public static SwerveModule[] swerveModules = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    /* Swerve class instance */
    public static Swerve swerve = new Swerve();
}
