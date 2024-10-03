// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.opencv.aruco.EstimateParameters;
import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;

import dev.doglog.AdvantageKitCompatibleLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.loging.BreakerLog;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.gtsam.GtsamInterface;
import frc.robot.subsystems.vision.gtsam.TagDetectionStruct;
import frc.robot.subsystems.vision.gtsam.TagDetection;

import static frc.robot.Constants.ApriltagVisionConstants.*;
import static frc.robot.Constants.FieldConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.naming.spi.DirObjectFactory;

import static frc.robot.Constants.DriveConstants.BACK_RIGHT_MODULE_CONSTANTS;

public class ApriltagVision extends SubsystemBase {
  /** Creates a new ApriltagVision. */  
  private static final Vector<N3> DOUBLE_MAX_VAL_VECTOR = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  private PhotonCamera frontLeftCam, frontRightCam, backLeftCam, backRightCam;
  private PhotonCamera[] cameras;
  private Transform3d[] cameraTransforms;
  private PhotonPoseEstimator[] poseEstimators;
  private VisionSystemSim visionSim;
  private GtsamInterface gtsam;
  private Drivetrain drivetrain;
  private ArrayList<Pair<EstimatedRobotPose, Vector<N3>>> estimatedPoses; 
  public ApriltagVision(GtsamInterface gtsam, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    frontLeftCam = new PhotonCamera(FRONT_LEFT_CAMERA_NAME);
      PhotonCameraSim frontLeftSim = new PhotonCameraSim(frontLeftCam, SIM_CAMERA_PROPERTIES);
    frontRightCam = new PhotonCamera(FRONT_RIGHT_CAMERA_NAME);
      PhotonCameraSim frontRightSim = new PhotonCameraSim(frontRightCam, SIM_CAMERA_PROPERTIES);
    backLeftCam = new PhotonCamera(BACK_LEFT_CAMERA_NAME);
      PhotonCameraSim backLeftSim = new PhotonCameraSim(backLeftCam, SIM_CAMERA_PROPERTIES);
    backRightCam = new PhotonCamera(BACK_RIGHT_CAMERA_NAME);
      PhotonCameraSim backRightSim = new PhotonCameraSim(backRightCam, SIM_CAMERA_PROPERTIES);

    cameras = new PhotonCamera[]{frontLeftCam, frontRightCam, backLeftCam, backRightCam};
    cameraTransforms = new Transform3d[]{FRONT_LEFT_CAMERA_TRANSFORM, FRONT_RIGHT_CAMERA_TRANSFORM, BACK_LEFT_CAMERA_TRANSFORM, BACK_RIGHT_CAMERA_TRANSFORM};
    poseEstimators = new PhotonPoseEstimator[cameras.length];
    
    visionSim = new VisionSystemSim("Apriltag_Vision_Sim");
    visionSim.addAprilTags(APRILTAG_FIELD_LAYOUT);
    visionSim.addCamera(frontLeftSim, FRONT_LEFT_CAMERA_TRANSFORM);
    visionSim.addCamera(frontRightSim, FRONT_RIGHT_CAMERA_TRANSFORM);
    visionSim.addCamera(backLeftSim, BACK_LEFT_CAMERA_TRANSFORM);
    visionSim.addCamera(backRightSim, BACK_RIGHT_CAMERA_TRANSFORM);

    estimatedPoses = new ArrayList<>();
    createPoseEstimators();
  }

  private void createPoseEstimators() {
    for (int i = 0; i < cameras.length; i++) {
      PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameras[i], cameraTransforms[i]);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
      poseEstimators[i] = poseEstimator;
      BreakerLog.log("ApriltagVision/Cameras" + cameras[i].getName() + "/IsConected", cameras[i].isConnected());
    }
  }

  private void logVisionFrame() {

  }

  private void updateSim(Pose2d robotPose) {
    visionSim.update(robotPose);
  }

  @Override
  public void periodic() {
    SwerveDriveState driveState = drivetrain.getState();
    estimatedPoses.clear();
    for (int i = 0; i < cameras.length; i++) {
      PhotonPoseEstimator estimator = poseEstimators[i];
      PhotonCamera camera = cameras[i];
      estimator.setReferencePose(driveState.Pose);
      Optional<EstimatedRobotPose> poseEstOpt  = estimator.update();
      if (poseEstOpt.isPresent()) {
        EstimatedRobotPose poseEst = poseEstOpt.get();
        List<PhotonTrackedTarget> trackedTargets = poseEst.targetsUsed;
        if (isInFieldBounds(poseEst.estimatedPose)) {
          if (trackedTargets.size() <= 1) {
            PhotonTrackedTarget tgt = trackedTargets.get(0);
            if (tgt.getPoseAmbiguity() >= 0.15) {
                continue;
            }
          }
          Vector<N3> stdDevs = calculateStdDevs(poseEst, driveState.speeds);
          if (!stdDevs.isEqual(DOUBLE_MAX_VAL_VECTOR, 1e-5)) {
            estimatedPoses.add(new Pair<EstimatedRobotPose,Vector<N3>>(poseEst, stdDevs));
            gtsam.setCamIntrinsics(camera.getName(), camera.getCameraMatrix(), camera.getDistCoeffs());
            gtsam.sendVisionUpdate(camera.getName(), (long)(poseEst.timestampSeconds * 1e6), toTagDetections(trackedTargets), estimator.getRobotToCameraTransform());
          }
        }
      }
    }
    sortByStandardDeviation(estimatedPoses);
    for (Pair<EstimatedRobotPose, Vector<N3>> estPose : estimatedPoses) {
      drivetrain.addVisionMeasurement(estPose.getFirst().estimatedPose.toPose2d(), estPose.getFirst().timestampSeconds, estPose.getSecond());
    }
  }

  private void sortByStandardDeviation(ArrayList<Pair<EstimatedRobotPose, Vector<N3>>> poses) {
    int i, j;
    Pair<EstimatedRobotPose, Vector<N3>> temp;
    boolean swapped;

    for (i = 0; i < poses.size() - 1; i++) {
        swapped = false;
        for (j = 0; j < poses.size() - i - 1; j++) {
            Pair<EstimatedRobotPose, Vector<N3>> jEstpos = poses.get(j);
            Pair<EstimatedRobotPose, Vector<N3>> j1Estpos = poses.get(j);

            if (jEstpos.getSecond().get(2, 0) < j1Estpos.getSecond().get(2, 0)) {
                temp = poses.get(j);
                poses.set(poses.indexOf(jEstpos), j1Estpos);
                poses.set(poses.indexOf(j1Estpos), temp);
                swapped = true;
            }
        }

        if (swapped == false)
            break;
    }
}

  private boolean isInFieldBounds(Pose3d estPos) {
    final Pose3d actual = estPos;
    final double fieldBorderMargin = 0.5;
    final double zMargin = 0.75;

    if (actual.getX() < -fieldBorderMargin
        || actual.getX() > APRILTAG_FIELD_LAYOUT.getFieldLength() + fieldBorderMargin
        || actual.getY() < -fieldBorderMargin
        || actual.getY() > APRILTAG_FIELD_LAYOUT.getFieldWidth() + fieldBorderMargin
        || actual.getZ() < -zMargin
        || actual.getZ() > zMargin) {
          return false;
        }
    return true;
  }

  private List<TagDetection> toTagDetections(List<PhotonTrackedTarget> trackedTargets) {
    ArrayList<TagDetection> dets = new ArrayList<>();
    for (PhotonTrackedTarget tgt: trackedTargets) {
      TagDetection det = new TagDetection(tgt.getFiducialId(), tgt.getDetectedCorners());
      dets.add(det);
    }
    return dets;
  }

  private Vector<N3> calculateStdDevs(EstimatedRobotPose posEst, ChassisSpeeds robotVels) {
      var estStdDevs = BASE_SINGLE_TAG_DEVS;
      var targets = posEst.targetsUsed;
      int numTags = 0;
      double avgDist = 0;
      for (var tgt : targets) {
        Pose3d tagPose = APRILTAG_FIELD_LAYOUT.getTagPose(tgt.getFiducialId()).orElseGet(() -> new Pose3d());
          numTags++;
          avgDist += 
                  tagPose.toPose2d().getTranslation().getDistance(posEst.estimatedPose.getTranslation().toTranslation2d());
      }
      if (numTags == 0) return estStdDevs;
      avgDist /= numTags;
      // Decrease std devs if multiple targets are visible
      if (numTags > 1) estStdDevs = BASE_MULTI_TAG_DEVS;
      // Increase std devs based on (average) distance
      if ((numTags == 1 && avgDist > MAX_SINGLE_TAG_DETECTION_DISTANCE.in(Units.Meters)) || (numTags > 1 && avgDist > MAX_MULTI_TAG_DETECTION_DISTANCE.in(Units.Meters)) )//4
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      else {estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / DISTANCE_INVERSE_SCAILAR));} //30

      double linVel = Math.hypot(robotVels.vxMetersPerSecond, robotVels.vyMetersPerSecond);
      estStdDevs.times(1 + (linVel * LINEAR_VEL_SCAILAR));
      estStdDevs.times(1 + (Math.abs(robotVels.omegaRadiansPerSecond) * ANGULAR_VEL_SCAILAR));
      
      return estStdDevs;
  }
}
