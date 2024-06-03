// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static java.lang.Math.round;

import java.io.InputStream;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.physics.BreakerVector3;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX flywheelLeft, flywheelRight;
  private TalonFX pivot;
  private CANcoder pivotEncoder;
  public Shooter() {
  }

  public void setState(Rotation2d piviotAngle, double flywheelAngularVelocity) {
      
  }

  // public Translation3d getFlywheelTranslationInRobotSpace(Rotation2d angle) {
  //   Translation3d flywheelInAxelSpace = PIVOT_AXEL_TO_FLYWHEEL_TRANS.rotateBy(new Rotation3d(0.0, -Math.PI + angle.getRadians(), 0.0));
  //   return ROBOT_TO_PIVOT_AXEL_TRANS.plus(flywheelInAxelSpace);
  // }

  // public Translation3d getFlywheelTranslationInFieldSpace(Pose3d robotPose, Rotation2d angle) {
  //   Translation3d robotSpaceTrans = getFlywheelTranslationInRobotSpace(angle);
  //   return robotSpaceTrans.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
  // }

  // public Translation3d getFlywheelTranslationInFieldSpace(Pose2d robotPose, Rotation2d angle) {
  //   return getFlywheelTranslationInFieldSpace(new Pose3d(robotPose), angle);
  // }
  public static record ShooterState(Rotation2d pitchAngle, Measure<Velocity<Angle>> flywheelVel) {
    public BreakerVector3 toNoteVelocity(Rotation2d launchYaw) {
      double velMPS = (flywheelVel.in(RadiansPerSecond) * FLYWHEEL_RADIUS.in(Units.Meters)) / NOTE_LAUCH_VEL_LOSS_SCAILAR;
      return new BreakerVector3(velMPS, new Rotation3d(0.0, pitchAngle.minus(Rotation2d.fromDegrees(180)).getRadians(), launchYaw.getRadians()));
    }

    public BreakerVector3 toNoteVelocity() {
      return toNoteVelocity(new Rotation2d());
    }
    
  }
  

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
