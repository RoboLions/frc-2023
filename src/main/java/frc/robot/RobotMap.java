package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

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
import frc.robot.lib.PhotonCameraWrapper;
import frc.robot.lib.RoboLionsPID;
import frc.robot.lib.Swerve;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class RobotMap {
    
    /* Motor + sensor IDs */
    public static final int pigeonID = 5;
    
    /* Motor + sensor instances */
    public static WPI_Pigeon2 gyro;

    /* Class instances */
    public static Swerve swerve; 
    public static RoboLionsPID rotationPID;
    public static CTREConfigs ctreConfigs;
    public static SwerveModule[] swerveModules;
    public static PhotonCamera camera;
    public static AprilTagFieldLayout aprilTagFieldLayout;
    public static PhotonPoseEstimator photonPoseEstimator;
    public static PhotonCameraWrapper pcw;
    public static SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    public static Field2d Field2d;

    /* Xbox controllers */
    public static XboxController manipulatorController;
    public static XboxController driverController;

    public static void init() {
        
        ctreConfigs = new CTREConfigs();
        manipulatorController = new XboxController(1);
        driverController = new XboxController(0);
        gyro = new WPI_Pigeon2(pigeonID);
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        rotationPID = new RoboLionsPID();
        swerve = new Swerve();
        camera = new PhotonCamera("Arducam_0V9281_USB_Camera");
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            var alliance = DriverStation.getAlliance();
            aprilTagFieldLayout.setOrigin(alliance == Alliance.Blue ?
                OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
            aprilTagFieldLayout = null;
        }
        pcw = new PhotonCameraWrapper();
        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.swerveKinematics, gyro.getRotation2d(), swerve.getModulePositions(), new Pose2d());
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, PhotonConstants.robotToCam);
        Field2d = new Field2d();
    }
}
