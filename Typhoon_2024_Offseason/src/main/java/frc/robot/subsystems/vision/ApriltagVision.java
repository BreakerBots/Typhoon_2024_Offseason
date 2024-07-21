// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.opencv.aruco.EstimateParameters;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.vision.gtsam.GtsamInterface;

import static frc.robot.Constants.ApriltagVisionConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.DriveConstants.BACK_RIGHT_MODULE_CONSTANTS;

public class ApriltagVision extends SubsystemBase {
  /** Creates a new ApriltagVision. */
  private PhotonCamera frontLeftCam, frontRightCam, backLeftCam, backRightCam;
  private VisionSystemSim visionSim;
  private GtsamInterface gtsam;
  private Drivetrain drivetrain;
  private PhotonPoseEstimator poseEstimator;
  public ApriltagVision(GtsamInterface gtsam) {
    frontLeftCam = new PhotonCamera(FRONT_LEFT_CAMERA_NAME);
      PhotonCameraSim frontLeftSim = new PhotonCameraSim(frontLeftCam, SIM_CAMERA_PROPERTIES);
    frontRightCam = new PhotonCamera(FRONT_RIGHT_CAMERA_NAME);
      PhotonCameraSim frontRightSim = new PhotonCameraSim(frontRightCam, SIM_CAMERA_PROPERTIES);
    backLeftCam = new PhotonCamera(BACK_LEFT_CAMERA_NAME);
      PhotonCameraSim backLeftSim = new PhotonCameraSim(backLeftCam, SIM_CAMERA_PROPERTIES);
    backRightCam = new PhotonCamera(BACK_RIGHT_CAMERA_NAME);
      PhotonCameraSim backRightSim = new PhotonCameraSim(backRightCam, SIM_CAMERA_PROPERTIES);
    
    visionSim = new VisionSystemSim("Apriltag_Vision_Sim");
    visionSim.addAprilTags(APRILTAG_FIELD_LAYOUT);
    visionSim.addCamera(frontLeftSim, FRONT_LEFT_CAMERA_TRANSFORM);
    visionSim.addCamera(frontRightSim, FRONT_RIGHT_CAMERA_TRANSFORM);
    visionSim.addCamera(backLeftSim, BACK_LEFT_CAMERA_TRANSFORM);
    visionSim.addCamera(backRightSim, BACK_RIGHT_CAMERA_TRANSFORM);
  }

  public void updateSim(Pose2d robotPose) {
    visionSim.update(robotPose);
  }

  @Override
  public void periodic() {
    
    
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
