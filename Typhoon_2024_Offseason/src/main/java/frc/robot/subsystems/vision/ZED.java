// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructDescriptor;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.physics.BreakerVector3;

public class ZED extends SubsystemBase {
  /** Creates a new ZED. */
  public ZED(Function<Double, Pose2d> robotPoseAtTimeFunc, Function<Double, ChassisSpeeds> chassisSpeedsAtTimeFunc) {

  }

  private ArrayList<TrackedObject> trackedObjects;

  public ArrayList<TrackedObject> getTrackedObjects() {
    return trackedObjects;
  }

  public Pose3d getCameraPose() {
    return null;
  }

  // public void test() {
  //   Transform3d robotVelsInWorldFrame = new Transform3d();
  //   Transform3d robotToCamera= new Transform3d();
  //   Translation3d objVelsInCameraFrame = new Translation3d();

  //   Translation3d cross = robotToCamera.rotateBy(robotVelsInWorldFrame.getRotation());
  //   Translation3d v_worldToObj = robotVelsInWorldFrame.getTranslation().plus(cross).plus(objVelsInCameraFrame);
  // }



  public static final record ObjectDimensions(double width, double height, double length) {}

  public static final class ObjectMotion {
    public ObjectMotion(double timestamp, BreakerVector3 observedCameraRelitiveObjectMotion, Transform3d robotToCameraTransform, Pose3d globalRobotPose) {

    }
  }

  public static final class ObjectPosition {
    public ObjectPosition(double timestamp, BreakerVector3 cameraRelivitveObjectVelocity, Translation3d cameraToObjectTranslation, Transform3d robotToCameraTransform, Pose2d globalRobotPose) {

      
    }

    public Translation3d getCameraToObject(boolean compensateForLatency) {
      return null;
    }

    public Translation3d getRobotToObject(boolean compensateForLatency) {
      return null;
    }

    public Translation3d getGlobal(boolean compensateForLatency) {
      return null;
    }
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public static final record TrackedObject(
    int objectID, 
    String label,
    double timestamp,
    BreakerVector3 velocity,
    ObjectPosition position,
    ObjectDimensions cameraRelitiveDimensions,
    double confidance,
    boolean isVisible,
    boolean isMoveing
  ) {} 
}
