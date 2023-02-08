package frc.robot;

import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.PhotonConstants;
import frc.robot.lib.states.Arm;
import frc.robot.lib.states.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RobotMap {
    
    /* Motor + sensor IDs */
    public static final int pigeonID = 5;
    public static final int leftBaseIntakeID = 6;
    public static final int rightBaseIntakeID = 7;
    public static final int intakeRollerID = 8;
    public static final int armFirstStageID = 9;
    public static final int armSecondStageID = 10;
    public static final int wristID = 11;
    public static final int clawID = 12;
    
    /* Motor + sensor instances */
    public static WPI_Pigeon2 gyro;
    public static WPI_TalonFX armFirstStage;
    public static WPI_TalonFX armSecondStage;
    public static WPI_TalonFX wrist;
    public static VictorSPX intakeRoller;
    public static VictorSPX claw;
    // TODO: tbd left and right base intake motors

    /* Class instances */
    public static Swerve swerve; 
    public static CTREConfigs ctreConfigs;
    public static AprilTagFieldLayout aprilTagFieldLayout;
    public static Field2d Field2d;
    public static Arm arm;

    /* Xbox controllers */
    public static XboxController manipulatorController;
    public static XboxController driverController;

    public static void init() {
        
        ctreConfigs = new CTREConfigs();
        manipulatorController = new XboxController(1);
        driverController = new XboxController(0);
        gyro = new WPI_Pigeon2(pigeonID);
        armFirstStage = new WPI_TalonFX(armFirstStageID);
        armSecondStage = new WPI_TalonFX(armSecondStageID);
        wrist = new WPI_TalonFX(wristID);
        intakeRoller = new VictorSPX(intakeRollerID);
        claw = new VictorSPX(clawID);
        swerve = new Swerve();
        arm = new Arm();
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            aprilTagFieldLayout = null;
        }
        Field2d = new Field2d();
    }
}
