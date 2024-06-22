// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static java.lang.Math.round;

import java.io.InputStream;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.BreakerLib.physics.BreakerVector3;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX flywheelLeft, flywheelRight;
  private TalonFX pivot;
  private CANcoder pivotEncoder;

  private VelocityVoltage flywheelVelocityRequest;
  private CoastOut flywheelCoastRequest;
  private Follower flywheelFollowRequest;
  private MotionMagicVoltage pitchMotionMagicRequest;

  private ShooterState setpoint;
  public Shooter() {
  }

  public void setState(ShooterState state) {
    setpoint = state.clamp();
    double flywheelVelSetpoint = setpoint.flywheelVel().in(Units.RevolutionsPerSecond);
    ControlRequest flywheelRequest = flywheelVelocityRequest.withVelocity(flywheelVelSetpoint);
    if (MathUtil.isNear(flywheelVelSetpoint, 0.0, 1e-5)) {
      flywheelRequest = flywheelCoastRequest;
    }
    flywheelLeft.setControl(flywheelRequest);
    flywheelRight.setControl(flywheelFollowRequest);
    pivot.setControl(pitchMotionMagicRequest.withPosition(setpoint.pitchAngle.getRotations()));
  }

  public Command setStateCommand(ShooterState state, boolean waitForSuccess) {
    
  }

  public Command runShooter(Supplier<ShooterState> stateSupplier) {
    return Commands.run(() -> setState(stateSupplier.get()), this);
  }

  public static record ShooterState(Rotation2d pitchAngle, Measure<Velocity<Angle>> flywheelVel) {
  
    public ShooterState withPitchAngle(Rotation2d newAngle) {
      return new ShooterState(newAngle, flywheelVel);
    }

    public ShooterState withFlywheelVel(Measure<Velocity<Angle>> newVel) {
      return new ShooterState(pitchAngle, newVel);
    }

    public ShooterState clamp() {
      double pitch = MathUtil.clamp(pitchAngle.getRadians(), Constants.ShooterConstants.MIN_ANGLE.getRadians(), Constants.ShooterConstants.MAX_ANGLE.getRadians());
      double flyVel = MathUtil.clamp(flywheelVel.in(Units.RadiansPerSecond), Constants.ShooterConstants.MIN_VEL.in(Units.RadiansPerSecond), Constants.ShooterConstants.MAX_VEL.in(Units.RadiansPerSecond));
      return new ShooterState(Rotation2d.fromRadians(pitch), Units.RadiansPerSecond.of(flyVel));
    }
  }


  public static record SmartSpoolConfig(Measure<Distance> flywheelSpoolDistance, Measure<Distance> pitchTrackDistance) {
  }
  
  public boolean atSetpoint() {
    return false;
  }

  public ShooterState getSetpoint() {
    return null;
  }

  public ShooterState getCurrentState() {
    return null;
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
