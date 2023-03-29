package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.interfaces.Swerve;
import frc.robot.lib.interfaces.SwerveModule;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.arm.ArmStateMachine;

public class TeleopState extends State {
    
    double translationVal;
    double strafeVal;
    double rotationVal = 0.0;
    double translationValScalar;

    double prevRotationVal = 0.0;
    double currRotationVal = 0.0;

    int count = 0;

    boolean accelEnabled = true;

    private SlewRateLimiter translationFilter = new SlewRateLimiter(2.0);
    private SlewRateLimiter strafeFilter = new SlewRateLimiter(2.0);

    private PIDController controller = new PIDController(0.1, 0.0, 0.0);

    public TeleopState() {}

    @Override
    public void build() {
        addTransition(new Transition(() -> {
            return RobotMap.driverController.getRawAxis(Constants.DriverControls.AUTO_ALIGN_AXIS) > 0.25;
        }, DrivetrainStateMachine.followTag));

        addTransition(new Transition(() -> {
            return RobotMap.driverController.getRawButton(Constants.DriverControls.AUTO_BALANCE_BUTTON);
        }, DrivetrainStateMachine.balanceState));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        // for(SwerveModule mod : Swerve.mSwerveMods) {
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        // }
        
        prevRotationVal = rotationVal;
        
        translationVal = -1.0 * MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.TRANSLATION_VAL), Constants.STICK_DEADBAND);
        strafeVal = -1.0 * MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.STRAFE_VAL), Constants.STICK_DEADBAND);
        rotationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.ROTATION_VAL), Constants.STICK_DEADBAND);

        // if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
        //     translationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.TRANSLATION_VAL), Constants.STICK_DEADBAND);
        //     strafeVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.STRAFE_VAL), Constants.STICK_DEADBAND);
        //     rotationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.ROTATION_VAL), Constants.STICK_DEADBAND);
        // } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
        //     translationVal = -1.0 * MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.TRANSLATION_VAL), Constants.STICK_DEADBAND);
        //     strafeVal = -1.0 * MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.STRAFE_VAL), Constants.STICK_DEADBAND);
        //     rotationVal = MathUtil.applyDeadband(-RobotMap.driverController.getRawAxis(Constants.DriverControls.ROTATION_VAL), Constants.STICK_DEADBAND);
        // }

        // anti-drifting
        if (rotationVal == 0.0 && prevRotationVal != 0.0) {
            double targetHeading = RobotMap.swerve.getYaw().getDegrees();
            count++;
            if (count > 100) {
                // the joystick input = 0 for a significant amount of time, meaning maintain the last target heading
                rotationVal = controller.calculate(RobotMap.swerve.getYaw().getDegrees(), targetHeading);
            }
        } else if (rotationVal != 0.0 && prevRotationVal == 0.0) {
            count = 0;
        }

        if (RobotMap.driverController.getRawButton(Constants.DriverControls.ZERO_GYRO)) {
            Translation2d current_coords = Swerve.swerveOdometry.getEstimatedPosition().getTranslation();
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
                RobotMap.swerve.resetOdometry(new Pose2d(current_coords, Rotation2d.fromDegrees(180.0)));
            }
            else {
                RobotMap.swerve.resetOdometry(new Pose2d(current_coords, Rotation2d.fromDegrees(0.0)));
            }
        }

        if (RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.groundPickupState) {
            translationValScalar = 2.0;
            rotationVal = rotationVal * Constants.SWERVE.MAX_ANGULAR_VELOCITY * 0.6;
        } else if (RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.scoreHighState || 
                   RobotMap.armStateMachine.getCurrentState() == ArmStateMachine.scoreMidState) {
            translationValScalar = 2.0;
            rotationVal = rotationVal * Constants.SWERVE.MAX_ANGULAR_VELOCITY * 0.3;
        } else {
            translationValScalar = Constants.SWERVE.MAX_SPEED;
            rotationVal = rotationVal * Constants.SWERVE.MAX_ANGULAR_VELOCITY;
        }

        // toggle whether the accel/decel on is enabled
        if (Swerve.bButton) {
            if (accelEnabled) {
                accelEnabled = false;
            } else {
                accelEnabled = true;
            }
        }

        // only apply the accel/decel filter if accelEnabled is true
        if (accelEnabled) {
            translationVal = translationFilter.calculate(translationVal);
            strafeVal = strafeFilter.calculate(strafeVal);
        }

        RobotMap.swerve.drive(
            new Translation2d(translationVal, strafeVal).times(translationValScalar), 
            rotationVal,
            true, 
            true
        );
    }

    @Override
    public void exit() {
        RobotMap.swerve.drive(
            new Translation2d(0, 0).times(Constants.SWERVE.MAX_SPEED), 
            0 * Constants.SWERVE.MAX_ANGULAR_VELOCITY, 
            true, 
            true
        );
    }
}
