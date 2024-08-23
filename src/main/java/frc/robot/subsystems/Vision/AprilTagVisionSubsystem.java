package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.awt.Desktop;
import java.net.URI;
import java.net.URISyntaxException;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotState;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;

public class AprilTagVisionSubsystem {
    public static PhotonCamera camAprTgLow;

    public static Alert camAprTgLowCamLatancyAlert = new Alert("April Tag Camera is experiencing high latancy", AlertType.WARNING);

    private static final AprilTagVisionSubsystem instance = new AprilTagVisionSubsystem();

    private static Field2d field;
    private static AprilTagFieldLayout fieldLayout;
    private static PhotonPoseEstimator camAprTgLowPoseEstimator;
    private static Transform3d camAprTgLowRobotToCam;
    private static Translation3d camAprTgLowRobotToCamTranslation;
    private static Rotation3d camAprTgLowRobotToCamRotation;
    private static VisionSystemSim visionSim;
    private static SimCameraProperties cameraProp;
    private static PhotonCameraSim simCamAprTgLow;
    
    static double longDistangePoseEstimationCount = 0;


    
    private AprilTagVisionSubsystem() {
        field = new Field2d();

        camAprTgLow = new PhotonCamera("camAprTgLow");

        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
        camAprTgLowRobotToCamTranslation = new Translation3d(Units.inchesToMeters(-1.0), Units.inchesToMeters(0.0), Units.inchesToMeters(12.0));
        camAprTgLowRobotToCamRotation = new Rotation3d(0, Units.degreesToRadians(-20), 0);
        camAprTgLowRobotToCam = new Transform3d(camAprTgLowRobotToCamTranslation, camAprTgLowRobotToCamRotation);
        camAprTgLowPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camAprTgLow, camAprTgLowRobotToCam);
        camAprTgLowPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if(Robot.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(fieldLayout);

            cameraProp = new SimCameraProperties();
            // A 1280 x 720 camera with a 70 degree diagonal FOV.
            cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
            // Approximate detection noise with average and standard deviation error in pixels.
            cameraProp.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            cameraProp.setFPS(30);
            // The average and standard deviation in milliseconds of image data latency.
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            simCamAprTgLow = new PhotonCameraSim(camAprTgLow, cameraProp);
            simCamAprTgLow.enableDrawWireframe(true);

            visionSim.addCamera(simCamAprTgLow, camAprTgLowRobotToCam);

            openSimCameraViews();
        }
    }

    public static AprilTagVisionSubsystem getInstance() {
        return instance;
    }

    /**
     * generates the estimated robot pose. Returns empty if:
     * <ul>
     *  <li> No Pose Estimates could be generated</li>
     * <li> The generated pose estimate was considered not accurate</li>
     * </ul>
     * @return an {@link EstimatedRobotPose} with an estimated pose, timestamp, and targets used to create the estimate
     */
    public static ArrayList<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> camAprTgLowPoseEst = filterPose(camAprTgLowPoseEstimator.update());
        
        ArrayList<EstimatedRobotPose> poses = new ArrayList<>();

        if (camAprTgLowPoseEst.isPresent()) {
            poses.add(camAprTgLowPoseEst.get());
            field.getObject("camAprTgLow est pose").setPose(camAprTgLowPoseEst.get().estimatedPose.toPose2d());
        }

        return poses;
    }

    private static Optional<EstimatedRobotPose> filterPose(Optional<EstimatedRobotPose> pose) {
        if(pose.isPresent()) {
            double bestTargetAmbiguity = 1; // 1 is max ambiguity
            for(PhotonTrackedTarget target : pose.get().targetsUsed) {
                double ambiguity = target.getPoseAmbiguity();
                if(ambiguity != -1 && ambiguity < bestTargetAmbiguity) bestTargetAmbiguity = ambiguity;
            }
            //ambiguity to high dont use estimate
            if(bestTargetAmbiguity > 0.3) return Optional.empty();

            //est pose is very far from recorded robot pose
            if(getDistanceFromPose(pose.get().estimatedPose.toPose2d()) > 1) {
                longDistangePoseEstimationCount++;

                //if it calculates that were 10 meter away for more than 10 times in a row its probably right
                if(longDistangePoseEstimationCount < 10) {
                    return Optional.empty();
                }
            } else longDistangePoseEstimationCount = 0;
            return pose;
        }
        return Optional.empty();
    }

    public static PhotonPipelineResult getLatestResult(Cameras camera) {
        PhotonCamera cam;
        PhotonCameraSim simCam;
        switch (camera) {
            case CAM_APR_TG_LOW: {
                cam = camAprTgLow;
                simCam = simCamAprTgLow;
                break;
            }
            default: {
                return null;
            }
        }

        if(Robot.isReal()) {
            return cam.getLatestResult();
        }
        return simCam.getCamera().getLatestResult();
    }

    public static boolean hasTargets(Cameras camera) {
        return getLatestResult(camera).hasTargets();
    }

    public static double getDistanceFromPose(Pose2d pose) {
        return PhotonUtils.getDistanceToPose(RobotState.robotPose, pose);
    }

    public static Pose2d getTagPose(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        if(tag.isPresent()) {
            return tag.get().toPose2d();
        }
        return null;
    }

    public static double getDistanceFromAprilTag(int id) {
        Optional<Pose3d> tag = fieldLayout.getTagPose(id);
        if(tag.isPresent()) {
            return getDistanceFromPose(tag.get().toPose2d());
        }
        return -1;
    }

    public static PhotonTrackedTarget getTargetFromId(int id, Cameras camera) {
        PhotonTrackedTarget target = null;
        PhotonPipelineResult result = getLatestResult(camera);
        if(result.hasTargets()) {
            for(PhotonTrackedTarget i : result.getTargets()) {
                if(i.getFiducialId() == id) {
                    target = i;
                }
            }
        }
        return target;
    }

    public static VisionSystemSim getVisionSim() {
        return visionSim;
    }

    private void openSimCameraViews() {
        if (Desktop.isDesktopSupported() && Desktop.getDesktop().isSupported(Desktop.Action.BROWSE)) {
            try {
                Desktop.getDesktop().browse(new URI("http://localhost:1182/"));
            } catch (IOException | URISyntaxException e) {
                e.printStackTrace();
            }
        }
    }

    public static Field2d getVisionField() {
        return field;
    }

    public static void updateVisionField() {
        SmartDashboard.putData("Vision/field", field);

        List<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();

        if(hasTargets(Cameras.CAM_APR_TG_LOW))targets.addAll(getLatestResult(Cameras.CAM_APR_TG_LOW).targets);

        List<Pose2d> poses = new ArrayList<>();
        for(PhotonTrackedTarget target : targets) {
            Pose2d targetPose = getTagPose(target.getFiducialId());
            poses.add(targetPose);
        }

        field.getObject("tracked targets").setPoses(poses);
        field.setRobotPose(RobotState.robotPose);

    }
    
    public enum Cameras {
        CAM_APR_TG_LOW    
    }

    public void periodic() {
        camAprTgLow.getLatestResult();
        SmartDashboard.putBoolean("camAprTgLow Connected",camAprTgLow.isConnected()); // Check if the camera is connected
        SmartDashboard.putBoolean("camAprTgLow has Targets",camAprTgLow.getLatestResult().hasTargets()); // Check if the camera has targets
    }
}
