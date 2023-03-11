package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.lib.util.SwerveModuleConstants;
import frc.robot.lib.util.COTSFalconSwerveConstants;

public final class Constants {
    public static final double STICK_DEADBAND = 0.25;

    public static final class CAN_IDS {
        /* Motor + sensor IDs */
        public static final int PIDGEON = 5;
        public static final int LEFT_SHOULDER_MOTOR = 50;
        public static final int RIGHT_SHOULDER_MOTOR = 51;
        public static final int LEFT_ELBOW_MOTOR = 53;
        public static final int RIGHT_ELBOW_MOTOR = 54;
        public static final int CLAW_MOTOR = 55;
    }

    public static final class PORTS {
        public static final I2C.Port COLOR_SENSOR = I2C.Port.kOnboard;
    }

    public static final class LIMELIGHT {
        public static final String NAME = "limelight";
    }

    public static final class SWERVE {
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.0);
        public static final double WHEEL_BASE = Units.inchesToMeters(22.0); 
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.9);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final boolean ANGLE_MOTOR_INVERT = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean DRIVE_MOTOR_INVERT = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERT = CHOSEN_MODULE.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
        public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = CHOSEN_MODULE.angleKP;
        public static final double ANGLE_KI = CHOSEN_MODULE.angleKI;
        public static final double ANGLE_KD = CHOSEN_MODULE.angleKD;
        public static final double ANGLE_KF = CHOSEN_MODULE.angleKF;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.05;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double DRIVE_KS = (0.49 / 12); 
        public static final double DRIVE_KV = (1.7 / 12);
        public static final double DRIVE_KA = (0.0 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 4.5; 
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 4.0;

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(59.90); // 235.72);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(37.2); // 215.50); 
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 30;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(350.50); // 169.80);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 41;
            public static final int angleMotorID = 42;
            public static final int canCoderID = 40;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(306.82); // 128.40);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final class Profile {
            // TODO: tune
            public static final PIDController X_CONTROLLER = new PIDController(1.0, 0, 0);
            public static final PIDController Y_CONTROLLER = new PIDController(1.0, 0, 0);
            public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(
                10.35,
                45.0,
                0.0, 
                new TrapezoidProfile.Constraints(0.5, 0.5)
            );
        }

        public static final class AUTO {
            /** Meters per Second */
            public static final double MAX_SPEED = 1.0; 
            /** Radians per Second */
            public static final double MAX_ANGULAR_VELOCITY = 1.0;
        }
    }

    public static final class PhotonConstants {
        // TODO: change to camera position on comp bot
        public static final Transform3d ROBOT_TO_CAM =
                new Transform3d(
                        new Translation3d(-1.0, 0.0, 0.0),
                        new Rotation3d(
                                0, 0,
                                0)); 
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
    }

    public static final class TargetPoses {
        // TODO: get poses with bot, 9 on red would = 1 one blue (they are flipped)

        public static Pose2d[] RED_SCORING_POSES = new Pose2d[]{
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0))
        };

        public static Pose2d[] BLUE_SCORING_POSES = new Pose2d[]{
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            new Pose2d(0.0, 0.0, new Rotation2d(0.0))
        };

        public static final Pose2d BLUE_LOADING_STATION = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
        public static final Pose2d RED_LOADING_STATION = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
    }

    // TODO: tune all and find positions of all below
    public static final class SHOULDER_MOTOR {
        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
        public static final double F_TRAVEL_LIMIT = 74000.0;
        public static final double B_TRAVEL_LIMIT = -10.0;
    }

    public static final class ELBOW_MOTOR {
        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
        public static final double F_TRAVEL_LIMIT = 104700.0;
        public static final double B_TRAVEL_LIMIT = -10.0;
    }

    public static final class CLAW {
        public static final double OPEN_POSITION = 500.0;
        public static final double CLOSED_POSITION = -80.0;
        public static final double ALLOWANCE = 10.0;
        public static final double TIME = 0.1;
        public static final double TIMEOUT = 1.0; // 1.2;

        public static final double OPEN_POWER = -0.8;
        public static final double CLOSE_POWER = 0.8;

        public static final Color CUBE_COLOR = new Color(0.21, 0.33, 0.46);
        public static final Color CONE_COLOR = new Color(0.37, 0.57, 0.00); // grayson cone: new Color(141, 93, 20);
    }

    public static final class OUTTAKE_STATE {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class GROUND_INTAKE {
        public static final double SHOULDER_POSITION = 13500.0;
        public static final double ELBOW_POSITION = 59500.0;
        public static final double ALLOWANCE = 100.0;
        public static final double TIME = 0.3;
    }

    public static final class HIGH_SCORE_CONE {
        public static final double SHOULDER_POSITION = 71500.0; 
        public static final double ELBOW_POSITION = 84300.0; 
        public static final double ALLOWANCE = 100.0;
        public static final double TIME = 0.5;
    }

    public static final class HIGH_SCORE_CUBE {
        public static final double SHOULDER_POSITION = 62000.0;
        public static final double ELBOW_POSITION = 80000.0;
        public static final double ALLOWANCE = 100.0;
        public static final double TIME = 0.5;
    }

    public static final class MID_SCORE_CONE {
        public static final double SHOULDER_POSITION = 44000.0;
        public static final double ELBOW_POSITION = 57000.0;
    }

    public static final class MID_SCORE_CUBE {
        public static final double SHOULDER_POSITION = 38400.0;
        public static final double ELBOW_POSITION = 68100.0;
    }

    public static final class LOW_SCORE_CONE {
        public static final double SHOULDER_POSITION = 12250.0;
        public static final double ELBOW_POSITION = 51000.0;
    }

    public static final class LOW_SCORE_CUBE {
        public static final double SHOULDER_POSITION = 12250.0;
        public static final double ELBOW_POSITION = 51000.0;
    }

    public static final class SUBSTATION_INTAKE {
        public static final double SHOULDER_POSITION = 66500.0;
        public static final double ELBOW_POSITION = 89500.0;
        public static final double ALLOWANCE = 100.0;
        public static final double TIME = 0.5;
    }

    public static final class ELBOW_IDLE {
        public static final double ELBOW_POSITION = 30000.0;
        public static final double ALLOWANCE = 10000.0;
        public static final double TIME = 0.2;
    }

    public static final class BALANCE_PITCH_PID {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class BALANCE_ROLL_PID {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class DriverControls {
        /*
         * SHIFT_LEFT_BUTTON:
         * in auto align state:
         *  if in a node scoring position, shifts the robot one node to the left
         */
        public static final int SHIFT_LEFT_BUTTON = XboxController.Button.kLeftBumper.value;
        /*
         * SHIFT_RIGHT_BUTTON:
         * same as above, but opposite direction
         */
        public static final int SHIFT_RIGHT_BUTTON = XboxController.Button.kRightBumper.value;
        /*
         * AUTO_ALIGN_BUTTON:
         * in teleop state:
         *  puts you in the auto align state
         */
        public static final int AUTO_ALIGN_BUTTON = XboxController.Button.kB.value;
        
        /*
         * Axes to control drive base in teleop mode
         */
        public static final int TRANSLATION_VAL = XboxController.Axis.kLeftY.value;
        public static final int STRAFE_VAL = XboxController.Axis.kLeftX.value;
        public static final int ROTATION_VAL = XboxController.Axis.kRightX.value;

        /*
         * in a score state:
         *  Kicks off the score piece sequence of events
         */
        public static final int SCORING_BUTTON = XboxController.Axis.kRightTrigger.value;

        /* in ground intake or substation intake:
         *  manually close the claw
         */
        public static final int CLOSE_BUTTON = XboxController.Axis.kRightTrigger.value;

        /*
         * in teleop state:
         *  Zeros the gyro of the robot and changes its field alignment
         */
        public static final int ZERO_GYRO = XboxController.Button.kX.value;
    }

    public static final class ManipulatorControls {
        /*
         * in any teleop state:
         *  puts the arm back into idle position
         */
        public static final int IDLE_BUTTON = XboxController.Button.kB.value;

        /*
         * in idle state:
         *  moves the arm into a scoring position, field orientation and color sensor determine diection
         *  and the exact position
         */
        public static final int HIGH_SCORE_BUTTON = XboxController.Button.kY.value;
        public static final int MID_SCORE_BUTTON = XboxController.Button.kX.value;
        public static final int LOW_SCORE_BUTTON = XboxController.Button.kA.value;
        
        /*
         * in idle state:
         *  moves arm into the ground intake position, front and back respectively
         */
        public static final int GROUND_INTAKE_FRONT = XboxController.Axis.kRightTrigger.value;

        /*
         * in idle state:
         *  sets the arm into the substation intake position, direction determined by field orientation
         */
        public static final int SUBSTATION_INTAKE_BUTTON = XboxController.Button.kRightBumper.value;

        /*
         * in manual control state:
         *  commands all the joints of the robot arm directly
         */
        public static final int ELBOW_AXIS = XboxController.Axis.kLeftY.value;
        public static final int SHOULDER_AXIS = XboxController.Axis.kRightY.value;

        /*
         * in idle state:
         *  command arm state machine into manual mode
         */
        public static final int MANUAL_MODE_BUTTON = XboxController.Button.kStart.value;
    }
}
