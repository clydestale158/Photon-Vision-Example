package frc.team3602.robot;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.math.util.Units.degreesToRadians;

import java.util.Optional;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Photon vision base class
 * Author: C Furmanski (clydestale158)
 * 
 * During a simulation, view camera view through a web browser by searching
 * localhost:1181 through localhost1184 (two ports are allocated per camera)
 */
public class Vision {
    /* Contants */
    public static final String CAM_1_NAME = "camera1";
    public static final String CAM_2_NAME = "camera2";

    public final static Transform3d ROBOT_TO_CAM_1 = new Transform3d(
            new Translation3d(0.33, 0.33, 0.02),
            new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(-15.0)));

    public final static Transform3d ROBOT_TO_CAM_2 = new Transform3d(
            new Translation3d(-0.33, 0, 0.02),
            new Rotation3d(0.0, degreesToRadians(15.0), degreesToRadians(180)));

    public final static PoseStrategy poseStrat = PoseStrategy.LOWEST_AMBIGUITY;

    /* Camera vars */
    private PhotonCamera camera1;
    private PhotonCamera camera2;

    /* Sim variables */
    private PhotonCameraSim camera1Sim;
    private PhotonCameraSim camera2Sim;

    private final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public VisionSystemSim visionSim = new VisionSystemSim("Vision System Viz");

    /* Pose Estimators */
    private final PhotonPoseEstimator camera1PE = new PhotonPoseEstimator(layout, poseStrat, ROBOT_TO_CAM_1);
    private final PhotonPoseEstimator camera2PE = new PhotonPoseEstimator(layout, poseStrat, ROBOT_TO_CAM_2);

    /* Data vars */ // things we want to use/read, must be updated perioidcally and will be the
                    // latest version
    public boolean target1Visible = false;
    public int target1Id = 0;
    public Transform3d target1Transform = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));
    public Pose2d camera1EstPose = new Pose2d(
            new Translation2d(0, 0),
            new Rotation2d(0, 0));
    public double camera1PETimeStamp = 0;

    public boolean target2Visible = false;
    public int target2Id = 0;
    public Transform3d target2Transform = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));
    public Pose2d camera2EstPose = new Pose2d(
            new Translation2d(0, 0),
            new Rotation2d(0, 0));
    public double camera2PETimeStamp = 0;

    /* Constructor, supports sim */
    public Vision() {
        if (RobotBase.isSimulation()) {
            /* camera sim setup */
            camera1Sim = new PhotonCameraSim(new PhotonCamera(CAM_1_NAME));
            camera1Sim.prop.setCalibError(0.08, 0.02);
            camera1Sim.prop.setFPS(10);
            camera1Sim.prop.setAvgLatencyMs(35);
            camera1Sim.prop.setLatencyStdDevMs(5);

            camera2Sim = new PhotonCameraSim(new PhotonCamera(CAM_2_NAME));
            camera2Sim.prop.setCalibError(0.08, 0.02);
            camera2Sim.prop.setFPS(10);
            camera2Sim.prop.setAvgLatencyMs(35);
            camera2Sim.prop.setLatencyStdDevMs(5);

            /* camera setup for while we are using sim */
            camera1 = camera1Sim.getCamera();
            camera2 = camera2Sim.getCamera();

            /* vision system sim setup */
            visionSim.addAprilTags(layout);
            visionSim.addCamera(camera1Sim, ROBOT_TO_CAM_1);
            visionSim.addCamera(camera2Sim, ROBOT_TO_CAM_2);
            SmartDashboard.putData("Vision System Viz", visionSim.getDebugField());
        } else {
            /* camera setup for real life */
            camera1 = new PhotonCamera(CAM_1_NAME);
            camera2 = new PhotonCamera(CAM_2_NAME);
        }
    }

    /** Must be called periodically in simulations */
    public void updateVisSim(Pose2d pose) {
        visionSim.update(pose);
        visionSim.getDebugField();
    }

    /** Must be called during initialization of simulations */
    public void resetVisSim() {
        visionSim.clearAprilTags();
        visionSim.addAprilTags(layout);
    }

    /** Must be called periodically! */
    public void updateReadings() {
        var results1 = camera1.getAllUnreadResults();
        Optional<EstimatedRobotPose> estPose1 = Optional.empty();

        if (results1.isEmpty()) { //sets data to default values 
            target1Visible = false;
            target1Id = 0;
            target1Transform = new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, 0, 0));
        } else {
            var result1 = results1.get(results1.size() - 1);
            if (result1.hasTargets()) {
                for (var target : result1.getTargets()) {
                    target1Visible = true;
                    target1Id = target.fiducialId;
                    target1Transform = target.bestCameraToTarget;

                    estPose1 = camera1PE.update(result1);
                    estPose1.ifPresent(est -> {
                        camera1EstPose = est.estimatedPose.toPose2d();
                        camera1PETimeStamp = est.timestampSeconds;
                    });

                    SmartDashboard.putString("Cam 1 readings", target.toString());
                    SmartDashboard.putNumber("Cam 1 PE X", camera1EstPose.getX());
                    SmartDashboard.putNumber("Cam 1 PE Y", camera1EstPose.getY());
                    SmartDashboard.putNumber("Cam 1 PE Rot", camera1EstPose.getRotation().getDegrees());
                    SmartDashboard.putNumber("Cam 1 PE Timestamp", camera1PETimeStamp);
                }
            }

        }

        var results2 = camera2.getAllUnreadResults();
        Optional<EstimatedRobotPose> estPose2 = Optional.empty();

        if (results2.isEmpty()) { //sets data to default values
            target2Visible = false;
            target2Id = 0;
            target2Transform = new Transform3d(
                    new Translation3d(0, 0, 0),
                    new Rotation3d(0, 0, 0));
        } else {
            var result2 = results2.get(results2.size() - 1);
            if (result2.hasTargets()) {
                for (var target : result2.getTargets()) {
                    target2Visible = true;
                    target2Id = target.fiducialId;
                    target2Transform = target.bestCameraToTarget;

                    estPose2 = camera2PE.update(result2);
                    estPose2.ifPresent(est -> {
                        camera2EstPose = est.estimatedPose.toPose2d();
                        camera2PETimeStamp = est.timestampSeconds;
                    });

                    SmartDashboard.putString("Cam 2 readings", target.toString());
                    SmartDashboard.putNumber("Cam 2 PE X", camera2EstPose.getX());
                    SmartDashboard.putNumber("Cam 2 PE Y", camera2EstPose.getY());
                    SmartDashboard.putNumber("Cam 2 PE Rot", camera2EstPose.getRotation().getDegrees());
                    SmartDashboard.putNumber("Cam 2 PE Timestamp", camera2PETimeStamp);
                }
            }
        }
    }

}