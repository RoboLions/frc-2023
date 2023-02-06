package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SwerveModule;
import frc.robot.lib.State;
import frc.robot.lib.Swerve;

public class TeleopState extends State {
    
    double translationVal;
    double strafeVal;
    double rotationVal;
    boolean robotCentric;

    public TeleopState() {}

    @Override
    public void init() {

        Swerve.gyro.configFactoryDefault();
        RobotMap.swerve.zeroGyro();

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        RobotMap.swerve.resetModulesToAbsolute();

        Swerve.swerveOdometry = RobotMap.swerveDrivePoseEstimator; // new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, RobotMap.swerve.getYaw(), RobotMap.swerve.getModulePositions());

    }

    @Override
    public void execute() {

        Swerve.swerveOdometry.update(RobotMap.swerve.getYaw(), RobotMap.swerve.getModulePositions());  

        for(SwerveModule mod : Swerve.mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        // invert because Xbox controllers return negative values when we push forward
        translationVal = MathUtil.applyDeadband(-Swerve.driverController.getLeftY(), Constants.stickDeadband);
        strafeVal = MathUtil.applyDeadband(-Swerve.driverController.getLeftX(), Constants.stickDeadband);
        // invert because Xbox controllers return positive values when you pull to the right
        rotationVal = MathUtil.applyDeadband(Swerve.driverController.getRightX(), Constants.stickDeadband);
        robotCentric = Swerve.driverController.getLeftBumperPressed();

        if (Swerve.driverController.getYButton()) {
            RobotMap.swerve.zeroGyro();
        }

        RobotMap.swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentric, 
            true
        );

    }

    @Override
    public void exit() {
        RobotMap.swerve.drive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            0 * Constants.Swerve.maxAngularVelocity, 
            !robotCentric, 
            true
        );
    }
}
