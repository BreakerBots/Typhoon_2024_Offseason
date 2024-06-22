// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructDescriptor;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.physics.BreakerVector3;

public class ZED extends SubsystemBase {
  /** Creates a new ZED. */
  public ZED() {
    
  }

  private ArrayList<TrackedObject> trackedObjects;

  public ArrayList<TrackedObject> getTrackedObjects() {
    return trackedObjects;
  }

  public static final record ObjectDimensions(double width, double height, double length) {}

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
