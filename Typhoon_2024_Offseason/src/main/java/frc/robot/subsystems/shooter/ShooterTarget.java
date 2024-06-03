package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static java.lang.Math.atan;
import static java.lang.Math.pow;

import java.io.InputStream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.HashMap;
import java.util.Set;
import java.util.Map.Entry;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import monologue.Annotations.IgnoreLogged;
import monologue.Annotations.Log;
import monologue.Logged;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterTarget implements Logged {
  public static final Measure<Distance> MAX_DISTANCE = Meters.of(5.0);
  public static final Measure<Angle> AIM_TOLERENCE_YAW = Degrees.of(2.0)

  private final Shooter shooter;
  private final Drivetrain drive;
  private final Translation3d targetPosition;
  public static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
  private final Function<Double, Double> flywheelSpeedInterpolationFunction;
  public ShooterTarget(Shooter shooter, HashMap<Measure<Distance>, Measure<Velocity<Angle>>> fireingTable) {
    this.shooter = shooter;

    double[] speeds = new double[fireingTable.size()];
    double[] dists = new double[fireingTable.size()];
    double minDist = Double.MAX_VALUE;
    double maxDist = -Double.MAX_VALUE;
    int i = 0;
    for (Entry<Measure<Distance>, Measure<Velocity<Angle>>> ent : fireingTable.entrySet()) {
      double dist = ent.getKey().in(Meters);
      if (dist > maxDist) {
        maxDist = dist;
      } 
      if (dist < minDist) {
        minDist = dist;
      }
      dists[i] = dist;
      speeds[i] = ent.getValue().in(RadiansPerSecond);
      i++;
    }
    PolynomialSplineFunction curve = SPLINE_INTERPOLATOR.interpolate(dists, speeds);
    flywheelSpeedInterpolationFunction = (Double dist) -> curve.value(MathUtil.clamp(dist, minDist, maxDist));
  }

  /**
   * Runs the shooter before feeding it the note.
   *
   * @param desiredVelocity The velocity in radians per second to shoot at.
   * @return The command to shoot at the desired velocity.
   */
  public Command shoot(Measure<Velocity<Angle>> desiredVelocity) {
    return shoot(() -> desiredVelocity.in(RadiansPerSecond), () -> true);
  }

  /**
   * Runs shooter to desired velocity, runs feeder once it reaches its velocity and shootCondition
   * is true.
   *
   * @param desiredVelocity Target velocity for the flywheel.
   * @param shootCondition Condition after which the feeder will run.
   */
  public Command shoot(DoubleSupplier desiredVelocity, BooleanSupplier shootCondition) {
    return Commands.waitUntil(() -> shooter.atSetpoint() && shootCondition.getAsBoolean())
        .andThen(feeder.eject())
        .deadlineWith(shooter.runShooter(desiredVelocity));
  }

  /**
   * Runs the pivot to an angle & runs the shooter before feeding it the note.
   *
   * @param targetAngle The desired angle of the pivot.
   * @param targetAngularVelocity The desired angular velocity of the shooter.
   * @return The command to run the pivot to its desired angle and then shoot.
   */
  public Command shootWithPivot(DoubleSupplier targetAngle, DoubleSupplier targetAngularVelocity) {
    return shoot(targetAngularVelocity, () -> pivot.atPosition(targetAngle.getAsDouble()))
        .deadlineWith(pivot.runPivot(targetAngle));
  }

  public Command shootWithPivot(
      Measure<Angle> targetAngle, Measure<Velocity<Angle>> targetVelocity) {
    return shootWithPivot(() -> targetAngle.in(Radians), () -> targetVelocity.in(RadiansPerSecond));
  }

  /** Shoots while stationary at correct flywheel speed and pivot angle, doesn't auto-turret. */
  public Command shootWithPivot() {
    return shootWithPivot(
        () -> pitchFromNoteVelocity(calculateNoteVelocity()),
        () -> rotationalVelocityFromNoteVelocity(calculateNoteVelocity()));
  }

  public Command aimWithoutShooting() {
    return pivot.runPivot(() -> pitchFromNoteVelocity(calculateNoteVelocity()));
  }

  /**
   * Shoots while driving at a manually inputted translational velocity.
   *
   * @param vx The field relative x velocity to drive in.
   * @param vy The field relative y velocity to drive in.
   * @return A command to shote while moving.
   */
  public Command shootWhileDriving(InputStream vx, InputStream vy) {
    return shoot(
            () -> rotationalVelocityFromNoteVelocity(calculateNoteVelocity()),
            () ->
                pivot.atPosition(pitchFromNoteVelocity(calculateNoteVelocity(Seconds.of(0.02))))
                    && atYaw(yawFromNoteVelocity(calculateNoteVelocity())))
        .deadlineWith(
            drive.drive(
                vx.scale(0.5),
                vy.scale(0.5),
                () -> yawFromNoteVelocity(calculateNoteVelocity(Seconds.of(0.2)))),
            pivot.runPivot(() -> pitchFromNoteVelocity(calculateNoteVelocity())));
  }

  public Pose2d robotPoseFacingSpeaker(Translation2d robotTranslation) {
    return new Pose2d(
        robotTranslation,
        translationToSpeaker(robotTranslation)
            .getAngle()
            .plus(Rotation2d.fromRadians(Math.PI)));
  }

  public Vector<N3> calculateNoteVelocity() {
    return calculateNoteVelocity(drive.getState().Pose);
  }

  public Vector<N3> calculateNoteVelocity(Measure<Time> predictionTime) {
    return calculateNoteVelocity(
        predictedPose(drive.pose(), drive.getFieldRelativeChassisSpeeds(), predictionTime));
  }

  /**
   * Calculates a vector for the desired note velocity relative to the robot for it to travel into
   * the speaker, accounting for the robot's current motion.
   *
   * @return A 3d vector representing the desired note initial velocity.
   */
  public Vector<N3> calculateNoteVelocity(Pose2d robotPose) {
    ChassisSpeeds speeds = drive.getFieldRelativeChassisSpeeds();
    Vector<N3> robotVelocity =
        VecBuilder.fill(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0);
    Translation2d difference = translationToSpeaker(robotPose.getTranslation());
    double shotVelocity = calculateStationaryVelocity(difference.getNorm());
    Rotation3d noteOrientation =
        new Rotation3d(
            0,
            -calculateStationaryPitch(
                robotPoseFacingSpeaker(robotPose.getTranslation()), shotVelocity, pivot.position()),
            difference.getAngle().getRadians());
    // rotate unit forward vector by note orientation and scale by our shot velocity
    Vector<N3> noteVelocity =
        new Translation3d(1, 0, 0).rotateBy(noteOrientation).toVector().unit().times(shotVelocity);

    return noteVelocity.minus(robotVelocity);
  }

  public static Pose2d predictedPose(
      Pose2d robotPose, ChassisSpeeds speeds, Measure<Time> predictionTime) {
    Vector<N3> current =
        VecBuilder.fill(robotPose.getX(), robotPose.getY(), robotPose.getRotation().getRadians());
    Vector<N3> velocity =
        VecBuilder.fill(
            speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    Vector<N3> predicted = current.plus(velocity.times(predictionTime.in(Seconds)));
    return new Pose2d(predicted.get(0), predicted.get(1), Rotation2d.fromRadians(predicted.get(2)));
  }

  /**
   * Returns the pose of the shooter.
   *
   * @return The pose of the shooter in 3d space.
   */
  @Log.NT
  public Pose3d shooterPose() {
    return new Pose3d(drive.pose())
        .transformBy(pivot.transform())
        .transformBy(PivotConstants.SHOOTER_FROM_AXLE);
  }


  public boolean isReady() {
    return shooter.atSetpoint() && pivot.atGoal();
  }

  /**
   * Calculates if the robot can make its current shot.
   *
   * @return Whether the robot can shoot from its current position at its current velocity.
   */
  @Log.NT
  public boolean inRange() {
    Vector<N3> shot = calculateNoteVelocity();
    double pitch = pitchFromNoteVelocity(shot);
    return MIN_ANGLE.in(Radians) < pitch
        && pitch < MAX_ANGLE.in(Radians)
        && Math.abs(rotationalVelocityFromNoteVelocity(shot)) < MAX_VELOCITY.in(RadiansPerSecond)
        && translationToSpeaker(drive.pose().getTranslation()).getNorm() < MAX_DISTANCE.in(Meters);
  }

  public boolean atYaw(Rotation2d yaw) {
    double tolerance = AIM_TOLERENCE_YAW.in(Radians) * (1 - yaw.getSin());
    Rotation2d diff = drive.getState().Pose.getRotation().minus(yaw);
    return Math.abs(atan(diff.getTan())) < tolerance;
  }

  /**
   * Calculates pitch from note initial velocity vector. If given a robot relative initial velocity
   * vector, the return value will also be the pivot angle.
   *
   * @param velocity Note initial velocity vector
   * @return Pitch/pivot angle
   */
  public static double pitchFromNoteVelocity(Vector<N3> velocity) {
    return Math.atan(velocity.get(2) / VecBuilder.fill(velocity.get(0), velocity.get(1)).norm());
  }

  /**
   * Calculates heading from note initial velocity vector. If given a robot relative initial
   * velocity vector, the return value will be the target robot heading.
   *
   * @param velocity Note initial velocity vector
   * @return Heading
   */
  public static Rotation2d yawFromNoteVelocity(Vector<N3> velocity) {
    return Rotation2d.fromRadians(Math.PI).plus(new Rotation2d(velocity.get(0), velocity.get(1)));
  }

  /**
   * Calculates magnitude of initial velocity vector of note, in radians per second. If given a
   * robot relative initial velocity vector, the return value will be the target flywheel speed
   * (ish).
   *
   * @param velocity Note initial velocity vector relative to the robot
   * @return Flywheel speed (rads / s)
   */
  public static double rotationalVelocityFromNoteVelocity(Vector<N3> velocity) {
    return velocity.norm() / FLYWHEEL_RADIUS.in(Meters) * NOTE_LAUCH_VEL_LOSS_SCAILAR;
  }

  /**
   * Converts between flywheel speed and note speed
   *
   * @param flywheelSpeed Flywheel speed in radians per second
   * @return Note speed in meters per second
   */
  public static double flywheelToNoteSpeed(double flywheelSpeed) {
    return flywheelSpeed * FLYWHEEL_RADIUS.in(Meters) / NOTE_LAUCH_VEL_LOSS_SCAILAR;
  }

  public Translation2d translationToSpeaker(Translation2d robotTranslation) {
    return targetPosition.toTranslation2d().minus(robotTranslation);
  }

  public double calculateStationaryVelocity(double distance) {
    return flywheelToNoteSpeed(flywheelSpeedInterpolationFunction.apply(distance));
  }

  /**
   * Calculates a stationary pitch from a pose so that the note goes into the speaker.
   *
   * @param shooterPose The pose of the shooter.
   * @param velocity The magnitude of velocity to launch the note at.
   * @return The pitch to shoot the note at.
   */
  public double calculateStationaryPitch(Pose2d robotPose, double velocity) {
    return calculateStationaryPitch(robotPose, velocity, getLinearShotAngle(robotPose), 0);
  }

  private Rotation2d getLinearShotAngle(Pose2d robotPose) {
    Pose3d pos3d = new Pose3d(robotPose);
    var fieldToAxle = ROBOT_TO_PIVOT_AXEL_TRANS.rotateBy(pos3d.getRotation()).plus(pos3d.getTranslation());
    double dist = targetPosition.toTranslation2d().getDistance(fieldToAxle.toTranslation2d());
    double h = targetPosition.getZ() - fieldToAxle.getZ();
    return new Rotation2d(dist, h);
  }

  private double calculateStationaryPitch(
      Pose2d robotPose, double velocity, Rotation2d guessPitch, int i) {
    double G = 9.81;
    Translation3d shooterTranslation = shooter.getFlywheelTranslationInFieldSpace(robotPose, guessPitch);
    double dist = translationToSpeaker(shooterTranslation.toTranslation2d()).getNorm();
    double h = targetPosition.getZ() - shooterTranslation.getZ();
    double denom = (G * pow(dist, 2));
    double rad =
        pow(dist, 2) * pow(velocity, 4)
            - G * pow(dist, 2) * (G * pow(dist, 2) + 2 * h * pow(velocity, 2));
    double pitch = Math.atan((1 / (denom)) * (dist * pow(velocity, 2) - Math.sqrt(rad)));
    if ((Math.abs(pitch - guessPitch.getRadians()) < 0.005 || i > 50) || !Constants.ShooterConstants.ENABLE_SOLUTION_ITTERATION) {
      return pitch;
    }
    return calculateStationaryPitch(robotPose, velocity, Rotation2d.fromRadians(pitch), i + 1);
  }
}