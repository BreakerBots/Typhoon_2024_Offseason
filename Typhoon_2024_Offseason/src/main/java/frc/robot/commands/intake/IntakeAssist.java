// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.Intake2.IntakeRollerState;
import frc.robot.subsystems.vision.NoteVision;
import frc.robot.subsystems.vision.ZED;
import frc.robot.subsystems.vision.NoteVision.TrackedNote2D;
import frc.robot.subsystems.vision.ZED.TrackedObject;

public class IntakeAssist extends Command {
  /** Creates a new IntakeAssist. */

  private static final double ANGLE_ERROR_SCAILAR = 1.0;
  private static final double DIST_ERROR_SCAILAR = 1.0;
  private static final double MAX_ERROR_SCORE = Double.MAX_VALUE;
  private static final double MAX_ANGLE_ERROR = 30.0 / 360.0;
  private static final double MAX_DIST_ERROR = 3.0;
  private static final double INPUT_MOVEING_AVERAGE_TIME = 0.35;
  private static final double TOTAL_DISENGAGE_OMEGA_THRESH = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond) * 0.2;
  private static final double OMEGA_ASSIST_DISEGAGE_THRESH = Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond) * 0.05;
  private static final double MIN_ACTIVE_TRANSLATIONAL_INPUT = 0.05;

  private Drivetrain drive;
  private Intake2 intake;
  private Hopper hopper;
  private ZED zed;
  private NoteVision nv;
  private BreakerInputStream xStream;
  private BreakerInputStream yStream;
  private BreakerInputStream xStreamFiltered;
  private BreakerInputStream yStreamFiltered;
  private BreakerInputStream omegaStream;
  private PIDController linearPID;
  private PIDController omegaPID;
  private SwerveRequest.FieldCentric driveRequest;
  
  public IntakeAssist(Drivetrain drive, Intake2 intake, Hopper hopper, ZED zed, NoteVision nv, BreakerInputStream xStream, BreakerInputStream yStream, BreakerInputStream omegaStream) {
    this.drive = drive;
    this.intake = intake;
    this.hopper = hopper;
    this.zed = zed;
    this.xStream = xStream;
    this.yStream = yStream;
    this.omegaStream = omegaStream;
    linearPID = new PIDController(0.0, 0.0, 0.0);
    omegaPID = new PIDController(0.0, 0.0, 0.0);
    omegaPID.enableContinuousInput(-0.5, 0.5);
    int taps = (int)(INPUT_MOVEING_AVERAGE_TIME * 50);
    xStreamFiltered = xStream.filter(LinearFilter.movingAverage(taps));
    yStreamFiltered = yStream.filter(LinearFilter.movingAverage(taps));
    driveRequest = new SwerveRequest.FieldCentric();
    driveRequest.DriveRequestType = DriveRequestType.Velocity;
    driveRequest.ForwardReference = ForwardReference.RedAlliance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOut = xStream.get();
    double yOut = yStream.get();
    double omegaOut = omegaStream.get();
    if (!nv.hasTarget()) {
      Optional<Pair<TrackedObject, Double>> bestNote = Optional.empty();
      SwerveDriveState state = drive.getState();
      BreakerVector2 filteredDriverReqVec = new BreakerVector2(xStreamFiltered.get(), yStreamFiltered.get());
      for (TrackedObject obj : zed.getTrackedObjects()) {
        if (obj.label() == "note" && obj.isVisible())  {
        double distErr = obj.position().getGlobal(true).toTranslation2d().getDistance(state.Pose.getTranslation());
        double errorBound = 0.5;
        double rawVecAngErr = filteredDriverReqVec.getVectorRotation().getRotations() - obj.position().getRobotToObject(true).toTranslation2d().getAngle().getRotations();
        double angErr = Math.abs(MathUtil.inputModulus(rawVecAngErr, -errorBound, errorBound));
        double errScore = (angErr * ANGLE_ERROR_SCAILAR) * (distErr * DIST_ERROR_SCAILAR);
        if (errScore <= MAX_ERROR_SCORE && angErr <= MAX_ANGLE_ERROR && distErr <= MAX_DIST_ERROR) {
            if ((bestNote.isPresent() && bestNote.get().getSecond() > errScore) || bestNote.isEmpty()) {
              bestNote = Optional.of(new Pair<TrackedObject, Double>(obj, errScore));
            }
        }
        }
      }
      
      if (bestNote.isPresent() && Math.abs(omegaOut) < TOTAL_DISENGAGE_OMEGA_THRESH && Math.hypot(xOut, yOut) >= MIN_ACTIVE_TRANSLATIONAL_INPUT ) {
        TrackedObject obj = bestNote.get().getFirst();
        double dist = obj.position().getGlobal(true).toTranslation2d().getDistance(state.Pose.getTranslation());
        var reqVec = new BreakerVector2(xOut, yOut).rotateBy(drive.getOperatorForwardDirection());
        double pidOut = linearPID.calculate(dist, 0.0);
        Rotation2d angToNote = BreakerMath.getPointAngleRelativeToOtherPoint(state.Pose.getTranslation(), obj.position().getGlobal(true).toTranslation2d());
        var pidVec = new BreakerVector2(angToNote, pidOut);
        var linearCorectionVec = reqVec.getUnitVector().minus(pidVec.getUnitVector()).times(pidOut);
        var linOutputVec = reqVec.plus(linearCorectionVec).clampMagnitude(0.0, Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond));
        xOut = linOutputVec.getX();
        yOut = linOutputVec.getY();
        if (omegaOut < OMEGA_ASSIST_DISEGAGE_THRESH) {
          omegaOut = omegaPID.calculate(angToNote.getRotations(), state.Pose.getRotation().getRotations());
        }
      }
    } else {
      TrackedNote2D tgt = nv.getBestTarget();
      omegaOut = omegaPID.calculate(nv)
    }
    driveRequest.withVelocityX(xOut).withVelocityY(yOut).withRotationalRate(omegaOut);
    drive.setControl(driveRequest);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getSetpoint().goalState().rollerState() != IntakeRollerState.INTAKEING || intake.hasNote() || hopper.hasNote();
  }
}
