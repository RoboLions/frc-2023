package frc.robot;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.lib.states.Arm;
import frc.robot.lib.states.Claw;
import frc.robot.lib.states.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Color;

public class RobotMap {
    
    /* Motor + sensor IDs */
    public static final int pigeonID = 5;
    public static final int shoulderMotorID = 9;
    public static final int elbowMotorID = 10;
    public static final int wristMotorID = 11;
    public static final int clawMotorID = 12;
    // TODO: change port
    public static final I2C.Port colorSensorPort = I2C.Port.kOnboard;

    /* Color instances */
    public static Color cubeColor;
    public static Color coneColor;
    public static ColorMatch colorMatcher;

    /* Motor + sensor instances */
    public static WPI_Pigeon2 gyro;
    public static WPI_TalonFX elbowMotor;
    public static WPI_TalonFX shoulderMotor;
    public static WPI_TalonFX wristMotor;
    public static VictorSPX clawMotor;
    // TODO: tbd intake motors
    public static ColorSensorV3 clawColorSensor;

    /* Class instances */
    public static Swerve swerve; 
    public static CTREConfigs ctreConfigs;
    public static AprilTagFieldLayout aprilTagFieldLayout;
    public static Field2d Field2d;
    public static Arm arm;
    public static Claw claw;

    /* Xbox controllers */
    public static XboxController manipulatorController;
    public static XboxController driverController;

    /* Claw open and close requests */
    public static boolean openRequest = false;
    public static boolean closeRequest = false;

    public static void init() {
        
        gyro = new WPI_Pigeon2(pigeonID);
        shoulderMotor = new WPI_TalonFX(shoulderMotorID);
        elbowMotor = new WPI_TalonFX(elbowMotorID);
        wristMotor = new WPI_TalonFX(wristMotorID);
        clawMotor = new VictorSPX(clawMotorID);
        ctreConfigs = new CTREConfigs();
        clawColorSensor = new ColorSensorV3(colorSensorPort);
        cubeColor = new Color(0.21, 0.33, 0.46);
        coneColor = new Color(0.37, 0.57, 0.00);
        colorMatcher = new ColorMatch();
        manipulatorController = new XboxController(1);
        driverController = new XboxController(0);
        swerve = new Swerve();
        arm = new Arm();
        claw = new Claw();
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            aprilTagFieldLayout = null;
        }
        Field2d = new Field2d();
    }
}
