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
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.lib.states.Swerve;

public class TeleopState extends State {
    public static XboxController driveController = RobotMap.driverController;
    double translationVal;
    double strafeVal;
    double rotationVal;
    boolean robotCentric;
    @Override
    public void build(){
        transitions.add(new Transition(() ->{
            return driveController.getXButton();
        }, DrivetrainStateMachine.balanceState));
        transitions.add(new Transition(()-> {
            return driveController.getBButton();
        }, DrivetrainStateMachine.aprilTagState));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {

        for(SwerveModule mod : Swerve.mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        // invert because Xbox controllers return negative values when we push forward
        translationVal = MathUtil.applyDeadband(-RobotMap.driverController.getLeftY(), Constants.stickDeadband);
        strafeVal = MathUtil.applyDeadband(-RobotMap.driverController.getLeftX(), Constants.stickDeadband);
        // invert because Xbox controllers return positive values when you pull to the right
        rotationVal = MathUtil.applyDeadband(RobotMap.driverController.getRightX(), Constants.stickDeadband);
        robotCentric = RobotMap.driverController.getLeftBumperPressed();

        if (RobotMap.driverController.getYButton()) {
            RobotMap.swerve.zeroGyro();
        }

        RobotMap.swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentric, 
            true
        );

    }
    }

