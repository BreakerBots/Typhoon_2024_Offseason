// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static java.lang.Math.round;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.math.BreakerUnits;

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
  private PositionVoltage pitchPositionRequest;

  private PitchControlType pitchControlType;
  private Rotation2d lastPitchSetpointOutsideOfEpsilon;

  private ShooterState setpoint;
  private Supplier<ChassisAccels> robotAccelSupplier;
  private Supplier<Double> flywheelVelSup;
  private Supplier<Double> flywheelAccelSup;
  private Supplier<Double> pivotPosSup;
  private Supplier<Double> pivotVelSup;
  public Shooter(Supplier<ChassisAccels> robotAccelSupplier) {
    this.robotAccelSupplier = robotAccelSupplier;
    pivot = new TalonFX(SHOOTER_PIVOT_ID, GeneralConstants.DRIVE_CANIVORE_NAME);
    flywheelLeft = new TalonFX(LEFT_FLYWHEEL_ID, GeneralConstants.DRIVE_CANIVORE_NAME);
    flywheelRight = new TalonFX(RIGHT_FLYWHEEL_ID, GeneralConstants.DRIVE_CANIVORE_NAME);
    pivotEncoder = BreakerCANCoderFactory.createCANCoder(PIVOT_ENCODER_ID, GeneralConstants.DRIVE_CANIVORE_NAME, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, PITCH_ENCODER_OFFSET, SensorDirectionValue.Clockwise_Positive);
    configPivot();
    configFlywheel();
    setState(getCurrentState());
  }

  private void configFlywheel() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = 10;
    config.CurrentLimits.SupplyCurrentThreshold = 100;
    config.CurrentLimits.SupplyTimeThreshold = 3.0;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    flywheelRight.getConfigurator().apply(config);
    config.Slot0.kP = 0.015;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.01;
    config.Slot0.kV = 0.1205;
    config.Slot0.kS = 0.16;
    config.Slot0.kA = 0.111;
    flywheelLeft.getConfigurator().apply(config);
    flywheelVelocityRequest= new VelocityVoltage(0.0);
    flywheelFollowRequest = new Follower(LEFT_FLYWHEEL_ID, true);
    flywheelCoastRequest = new CoastOut();
    flywheelVelSup = flywheelLeft.getVelocity().asSupplier();
    flywheelAccelSup = flywheelLeft.getAcceleration().asSupplier();
  }

  private void configPivot() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackRemoteSensorID = PIVOT_ENCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.RotorToSensorRatio = PIVOT_RATIO.getRatioToOne();
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kP = PIVOT_MOTION_MAGIC_kP;
    config.Slot0.kI = PIVOT_MOTION_MAGIC_kI;
    config.Slot0.kD = PIVOT_MOTION_MAGIC_kD;
    config.Slot0.kV = PIVOT_kV;
    config.Slot0.kS = PIVOT_kS;
    config.Slot0.kA = PIVOT_kA;
    config.Slot0.kG = PIVOT_kG;
    config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot1.kP = PIVOT_POSITION_kP;
    config.Slot1.kI = PIVOT_POSITION_kI;
    config.Slot1.kD = PIVOT_POSITION_kD;
    config.Slot1.kV = PIVOT_kV;
    config.Slot1.kS = PIVOT_kS;
    config.Slot1.kA = PIVOT_kA;
    config.Slot1.kG = PIVOT_kG;
    config.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

    config.MotionMagic.MotionMagicCruiseVelocity = 4.0;
    config.MotionMagic.MotionMagicAcceleration = 0.6;

    config.CurrentLimits.SupplyCurrentLimit = 80;
    config.CurrentLimits.SupplyTimeThreshold = 1.5;
    config.CurrentLimits.SupplyCurrentLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.getRotations();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.getRotations();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pitchMotionMagicRequest = new MotionMagicVoltage(0.0, true, 0.0, 0, false, false, false);
    pitchPositionRequest = new PositionVoltage(0.0, 0.0, true, 0.0, 1, false, false, false);
    pivot.getConfigurator().apply(config);
    pivotPosSup = pivot.getPosition().asSupplier();
    pivotVelSup = pivot.getVelocity().asSupplier();
  }


  private static enum PitchControlType {
    POSITION,
    MOTION_MAGIC
  }

  private void setState(ShooterState state) {
    state = state.clamp();
    double flywheelVelSetpoint = state.flywheelVel().in(Units.RevolutionsPerSecond);
    ControlRequest flywheelRequest = flywheelVelocityRequest.withVelocity(flywheelVelSetpoint);
    if (MathUtil.isNear(flywheelVelSetpoint, 0.0, 1e-5)) {
      flywheelRequest = flywheelCoastRequest;
    }
    flywheelLeft.setControl(flywheelRequest);
    flywheelRight.setControl(flywheelFollowRequest);

    boolean oustideControlTypeSwitchThreshFromPrev = !MathUtil.isNear(state.pitchAngle.getRotations(), lastPitchSetpointOutsideOfEpsilon.getRotations(), PITCH_SETPOINT_CONTROL_TYPE_SWITCH_EPSILON.in(Units.Rotations), -0.5, 0.5);
    if (oustideControlTypeSwitchThreshFromPrev) {
      lastPitchSetpointOutsideOfEpsilon = state.pitchAngle;
    }

    boolean isToCloseForMotionMagic = MathUtil.isNear(state.pitchAngle.getRotations(), getCurrentState().pitchAngle.getRotations(), MOTION_MAGIC_MIN_DIST.in(Units.Rotations), -0.5, 0.5);;
    
    double robotAccelFeedforward = robotAccelSupplier.get().axMetersPerSecondSquared * PIVOT_kX;

    if (isToCloseForMotionMagic && ((oustideControlTypeSwitchThreshFromPrev && pitchControlType == PitchControlType.MOTION_MAGIC) || pitchControlType == PitchControlType.POSITION)) {
      pivot.setControl(pitchPositionRequest.withPosition(state.pitchAngle.getRotations()).withFeedForward(robotAccelFeedforward));
      pitchControlType = PitchControlType.POSITION;
    } else {
      pivot.setControl(pitchMotionMagicRequest.withPosition(state.pitchAngle.getRotations()).withFeedForward(robotAccelFeedforward));
      pitchControlType = PitchControlType.MOTION_MAGIC;
    }
    setpoint = state;
  }

  public Command setStateCommand(ShooterState state, boolean waitForSuccess) {
    return Commands.runOnce(() -> setState(state), this).andThen(new WaitUntilCommand(() -> {return atSetpoint() || !waitForSuccess;}));
  }

  public Command runShooter(Supplier<ShooterState> stateSupplier) {
    return Commands.run(() -> setState(stateSupplier.get()), this);
  }

  public Command runShooter(Supplier<ShooterState> stateSupplier, Subsystem... subsystems) {
    ArrayList<Subsystem> reqs = new ArrayList<>(Arrays.asList(subsystems));
    reqs.add(this);
    Subsystem[] reqArr = reqs.toArray(new Subsystem[reqs.size()]);
    return Commands.run(() -> setState(stateSupplier.get()), reqArr);
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

  public boolean atPivotSetpoint() {
    ShooterState curState = getCurrentState();
    boolean atPos = MathUtil.isNear(setpoint.pitchAngle.getRotations(), curState.pitchAngle.getRotations(), PIVOT_POS_TOLERENCE.in(Units.Rotations), -0.5, 0.5);
    boolean atVel = MathUtil.isNear(0.0, pivotVelSup.get(), PIVOT_VEL_TOLERENCE.in(Units.RotationsPerSecond));
    return atPos && atVel;
  }

  public boolean atFlywheelSetpoint() {
    ShooterState curState = getCurrentState();
    boolean atVel = MathUtil.isNear(setpoint.flywheelVel.in(Units.RotationsPerSecond), curState.flywheelVel.in(Units.RotationsPerSecond), FLYWHEEL_VEL_TOLERENCE.in(Units.RotationsPerSecond));
    boolean atAccel = MathUtil.isNear(0.0, flywheelAccelSup.get(), FLYWHEEL_ACCEL_TOLERENCE.in(BreakerUnits.RotationsPerSecondPerSecond));
    return atVel && atAccel;
  }
  
  public boolean atSetpoint() {
    return atPivotSetpoint() && atFlywheelSetpoint();
  }

  public ShooterState getSetpoint() {
    return setpoint;
  }

  public ShooterState getCurrentState() {
    return new ShooterState(Rotation2d.fromRotations(pivotPosSup.get()), Units.RotationsPerSecond.of(flywheelVelSup.get()));
  }


  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }
}
