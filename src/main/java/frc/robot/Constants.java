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
        public static final int SHOULDER_MOTOR = 9;
        public static final int ELBOW_MOTOR = 10;
        public static final int WRIST_MOTOR = 11;
        public static final int CLAW_MOTOR = 12;
    }

    public static final class PORTS {
        // TODO: change port
        public static final I2C.Port COLOR_SENSOR = I2C.Port.kOnboard;
    }

    public static final class SWERVE {
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants CHOSEN_MODULE = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(23.5);
        public static final double WHEEL_BASE = Units.inchesToMeters(23.5); 
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.875);
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
        public static final boolean angleMotorInvert = CHOSEN_MODULE.angleMotorInvert;
        public static final boolean driveMotorInvert = CHOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = CHOSEN_MODULE.canCoderInvert;

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
        public static final double angleKP = CHOSEN_MODULE.angleKP;
        public static final double angleKI = CHOSEN_MODULE.angleKI;
        public static final double angleKD = CHOSEN_MODULE.angleKD;
        public static final double angleKF = CHOSEN_MODULE.angleKF;

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
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(236.68);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 21;
            public static final int angleMotorID = 22;
            public static final int canCoderID = 20;
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

        public static final class Profile {
            public static final PIDController X_CONTROLLER = new PIDController(0.01, 0, 0);
            public static final PIDController Y_CONTROLLER = new PIDController(0.01, 0, 0);
            public static final ProfiledPIDController THETA_CONTROLLER = new ProfiledPIDController(
                0.01,
                0,
                0, 
                new TrapezoidProfile.Constraints(0.25, 0.25)
            );
        }
    }

    public static final class PhotonConstants {
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

    public static final class ShoulderMotorConstants {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class ElbowMotorConstants {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class WristMotorConstants {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class Claw {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;

        public static final double CLOSED_CUBE_POSITION = 0.0;
        public static final double CLOSED_CONE_POSITION = 0.0;

        public static final double TIME_CLOSE_ON_CONE = 0.5;
        public static final double TIME_CLOSE_ON_CUBE = 0.1;
        public static final double TIME_OPEN_CLAW = 0.1;

        public static final Color CUBE_COLOR = new Color(0.21, 0.33, 0.46);
        public static final Color CONE_COLOR = new Color(0.37, 0.57, 0.00);
    }

    public static final class BHighPurple {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }
    
    public static final class BHighYellow {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class BHybrid {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class BMidPurple {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class BMidYellow {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    
    public static final class FHighPurple {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }
    
    public static final class FHighYellow {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class FHybrid {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class FMidPurple {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class FMidYellow {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class FIntakeState {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
    }

    public static final class BIntakeState {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
    }

    public static final class OuttakeState {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
        public static final double ALLOWANCE = 0.0;
        public static final double TIME = 0.0;
    }

    public static final class BPickupState {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
    }

    public static final class FPickupState {
        public static final double SHOULDER_POSITION = 0.0;
        public static final double ELBOW_POSITION = 0.0;
        public static final double WRIST_POSITION = 0.0;
    }

    public static final class BalancePitchPID {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class BalanceRollPID {
        public static final double P = 0.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
    }

    public static final class DriverButtons {
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
         * in teleop state:
         *  puts robot into auto balace state
         */
        public static final int AUTO_BALANCE_BUTTON = XboxController.Button.kX.value;

        /*
         * in a score state:
         *  Kicks off the score biece sequence of events
         */
        public static final int SCORING_BUTTON = XboxController.Axis.kRightTrigger.value;

        /*
         * in teleop state:
         *  Zeros the gyro of the robot and changes it's field alignment
         */
        public static final int ZERO_GYRO = XboxController.Axis.kLeftTrigger.value;
    }

    public static final class ManipulatorButtons {
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
         *  moves arm into the ground intake position, front and back respectivley
         */
        public static final int GROUND_INTAKE_BACK = XboxController.Axis.kLeftTrigger.value;
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
        public static final int WRIST_FORWARD_BUTTON = XboxController.Axis.kLeftTrigger.value;
        public static final int WRIST_BACKWARD_BUTTON = XboxController.Axis.kRightTrigger.value;
        public static final int BICEP_BUTTON = XboxController.Axis.kLeftY.value;
        public static final int FOREARM_BUTTON = XboxController.Axis.kRightY.value;

        /*
         * in idle state:
         *  command arm state machine into manual mode
         */
        public static final int MANUAL_MODE_BUTTON = XboxController.Button.kStart.value;

        /*
         * in idle mode:
         *  sets the arm position into the endgame position
         */
        public static final int ENDGAME_BUTTON = XboxController.Button.kBack.value;

        // outtake button is in OuttakeState.java because it is a POV
    }
}
