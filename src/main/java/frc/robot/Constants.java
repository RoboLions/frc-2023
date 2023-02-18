package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.lib.util.COTSFalconSwerveConstants;

public final class Constants {
    public static final double stickDeadband = 0.25;

    public static final class Swerve {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.5);
        public static final double wheelBase = Units.inchesToMeters(23.5); 
        public static final double wheelDiameter = Units.inchesToMeters(3.875);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.15;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.68 / 12); 
        public static final double driveKV = (0.0 / 12);
        public static final double driveKA = (0.0 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(236.68);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(349.36);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 30;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(37.62);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 41;
            public static final int angleMotorID = 42;
            public static final int canCoderID = 40;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(129.11);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; //3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 1.5; //Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1.5; //Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    
    public static final class PhotonConstants {
        public static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(-1.0, 0.0, 0.0),
                        new Rotation3d(
                                0, 0,
                                0)); 
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
    }

    public static final class ArmFirstStageConstants {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class ArmSecondStageConstants {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class Wrist {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class Claw {
        // TODO: change time
        public static final double clawClosedCubeTime = 0.25;
        public static final double clawClosedConeTime = 0.5;
    }

    public static final class BHighPurple {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }
    
    public static final class BHighYellow {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    public static final class BHybrid {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    public static final class BMidPurple {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    public static final class BMidYellow {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    
    public static final class FHighPurple {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }
    
    public static final class FHighYellow {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    public static final class FHybrid {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    public static final class FMidPurple {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    public static final class FMidYellow {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    public static final class IntakeState {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
    }

    public static final class OuttakeState {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
        public static final double ALLOWANCE;
        public static final double TIME;
    }

    public static final class PickupState {
        public static final double FIRST_STAGE_POSITION;
        public static final double SECOND_STAGE_POSITION;
        public static final double WRIST_POSITION;
    }
}