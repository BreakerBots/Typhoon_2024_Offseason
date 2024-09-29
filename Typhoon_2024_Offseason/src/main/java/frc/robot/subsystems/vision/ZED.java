// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.CoordinateAxis;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.WPINetJNI;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructDescriptor;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.vision.ZED.TrackedObject;

public class ZED extends SubsystemBase {
  /** Creates a new ZED. */
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("ZED_detections");
  private IntegerSubscriber heartbeatSub = table.getIntegerTopic("heartbeat").subscribe(-1);
  private IntegerArraySubscriber idSub = table.getIntegerArrayTopic("id").subscribe(new long[0]);
  private StringArraySubscriber labelSub = table.getStringArrayTopic("label").subscribe(new String[0]);
  private IntegerSubscriber latencySub = table.getIntegerTopic("pipeline_latency").subscribe(0);
  private DoubleArraySubscriber xVelPub = table.getDoubleArrayTopic("x_vel").subscribe(new double[0]);
  private DoubleArraySubscriber yVelPub = table.getDoubleArrayTopic("y_vel").subscribe(new double[0]);
  private DoubleArraySubscriber zVelPub = table.getDoubleArrayTopic("z_vel").subscribe(new double[0]);
  private DoubleArraySubscriber xPub = table.getDoubleArrayTopic("x").subscribe(new double[0]);
  private DoubleArraySubscriber yPub = table.getDoubleArrayTopic("y").subscribe(new double[0]);
  private DoubleArraySubscriber zPub = table.getDoubleArrayTopic("z").subscribe(new double[0]);
  private DoubleArraySubscriber boxLenPub = table.getDoubleArrayTopic("box_l").subscribe(new double[0]);
  private DoubleArraySubscriber boxWidthPub = table.getDoubleArrayTopic("box_w").subscribe(new double[0]);
  private DoubleArraySubscriber boxHeightPub = table.getDoubleArrayTopic("box_h").subscribe(new double[0]);
  private DoubleArraySubscriber confPub = table.getDoubleArrayTopic("conf").subscribe(new double[0]);
  private BooleanArraySubscriber isVisPub = table.getBooleanArrayTopic("is_visible").subscribe(new boolean[0]);
  private BooleanArraySubscriber isMovingPub = table.getBooleanArrayTopic("is_moving").subscribe(new boolean[0]);
  private long lastHeartbeat = -1;
  private Timer timeSinceLastUpdate;

  private Transform3d robotToZedLeftEye;
  private CoordinateSystem coordinateSystem;

  public ZED(Function<Double, Pose2d> robotPoseAtTimeFunc, Function<Double, ChassisSpeeds> chassisSpeedsAtTimeFunc) {

    coordinateSystem = BreakerMath.getCoordinateSystemFromRotation(robotToZedLeftEye.getRotation());
  }

  
  public Transform3d getRobotToCameraTransform() {
    return robotToZedLeftEye;
  }

  private ArrayList<TrackedObject> trackedObjects;

  public ArrayList<TrackedObject> getTrackedObjects() {
    return trackedObjects;
  }

  public Pose3d getCameraPose() {
    return null;
  }


  public static final record ObjectDimensions(double width, double height, double length) {}

  public static final class ObjectMotion {
    private BreakerVector3 observedCameraRelativeObjectMotion;
    private CoordinateSystem cameraCoordinateSystem;
    private Pose3d globalRobotPose;
    ObjectMotion(double timestamp, 
    BreakerVector3 observedCameraRelativeObjectMotion, 
    Pose3d globalRobotPose,
    ChassisSpeeds robotSpeedsInWorld,
    CoordinateSystem cameraCoordinateSystem) {

    }
    
    public BreakerVector3 getCameraRelative () {
      return observedCameraRelativeObjectMotion;
    }

    public BreakerVector3 getRobotRelative() {
      return new BreakerVector3(CoordinateSystem.convert(getCameraRelative().getAsTranslation(), cameraCoordinateSystem, CoordinateSystem.NWU()));
    }

    public BreakerVector3 getGlobal() {
      CoordinateSystem robotSys = BreakerMath.getCoordinateSystemFromRotation(globalRobotPose.getRotation());
      Translation3d robotRel = getRobotRelative().getAsTranslation();
      Translation3d rawWorld = CoordinateSystem.convert(robotRel, robotSys, CoordinateSystem.NWU()).minus(robotRel);
      
      return new BreakerVector3();
    }
  }
  

  public static final class ObjectPosition {
    private double timestamp;
    private ObjectMotion motion;
    private Translation3d cameraToObjectTranslation;
    public ObjectPosition(double timestamp, ObjectMotion motion, Translation3d cameraToObjectTranslation, Transform3d robotToCameraTransform, Pose2d globalRobotPose) {

      
    }

    public Translation3d getCameraToObject(boolean compensateForLatency) {
      double latency = Timer.getFPGATimestamp() - timestamp;
      Translation3d pos = cameraToObjectTranslation;
      if (compensateForLatency) {
        Translation3d trans = motion.getCameraRelative().times(latency).getAsTranslation();
        pos = pos.plus(trans);
      }
      return pos;
    }

    public Translation3d getRobotToObject(boolean compensateForLatency) {
      Translation3d camRelTrans = getCameraToObject(false);
      camRelTrans.rotateBy(null);
      return null;
    }

    public Translation3d getGlobal(boolean compensateForLatency) {
      return null;
    }
  }

  @Override
  public void periodic() {
    long newHb = heartbeatSub.get();
    if (newHb > lastHeartbeat && newHb != -1) {
      timeSinceLastUpdate.reset();
    }
    // This method will be called once per scheduler run
  }

  private void readTargets() {
    long[] ids = idSub.get();
    String[] lables = labelSub.get();
    TimestampedInteger latency = latencySub.getAtomic();
    double captureTimestamp = (((double)(latency.timestamp)) - (((double)(latency.value)) / 1000.0)) / ((double)(1e6));
    
    for (int i = 0; i < ids.length; i++) {
      
    }
  }

  public static final record TrackedObject(
      long objectID, 
      String label,
      double timestamp,
      ObjectMotion motion,
      ObjectPosition position,
      ObjectDimensions cameraRelitiveDimensions,
      double confidance,
      boolean isVisible,
      boolean isMoveing) implements Comparable<TrackedObject> {

    @Override
    public int compareTo(TrackedObject otherTracked) {
        return (int) (MathUtil.clamp(otherTracked.objectID - objectID, (long) Integer.MIN_VALUE, (long) Integer.MAX_VALUE));
    }

    @Override
    public boolean equals(Object object) {
      if (object instanceof TrackedObject) {
        var otherTracked = (TrackedObject) object;
        return otherTracked.objectID == objectID;
      }
      return false;
    }
    
  } 
}
