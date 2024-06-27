// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.swerve;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.proto.Twist2dProto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.subsystems.Hopper;
import monologue.Logged;
import monologue.Annotations.Log;

public class BreakerSwerveDrivetrain extends SwerveDrivetrain implements Subsystem, Logged {

  /** Creates a new BreakerSwerveDrivetrain. */
  protected Notifier simNotifier = null;
  protected double lastSimTime;
  protected final double commonMaxModuleSpeed;
  protected final BreakerSwerveDrivetrainConstants constants;
  protected final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
  protected Consumer<SwerveDriveState> userTelemetryCallback = null;
  /* Keep track if we've ever applied the operator perspective before or not */
  protected boolean hasAppliedOperatorPerspective = false;

  public BreakerSwerveDrivetrain(
    BreakerSwerveDrivetrainConstants driveTrainConstants, 
    SwerveModuleConstants... modules
  ) {
    this(driveTrainConstants,
    VecBuilder.fill(0.1, 0.1, 0.1),
    VecBuilder.fill(0.9, 0.9, 0.9), 
    modules);
  }

  public BreakerSwerveDrivetrain(
      BreakerSwerveDrivetrainConstants driveTrainConstants, 
      Matrix<N3, N1> odometryStandardDeviation, 
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules
    ) {
    super(driveTrainConstants, driveTrainConstants.odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    this.constants = driveTrainConstants;
    m_telemetryFunction = this::telemetryCallbackWrapperFunction;
    double tempCommonMaxModuleSpeed = Double.MAX_VALUE;
    for (SwerveModuleConstants modConst : modules) {
      tempCommonMaxModuleSpeed = Math.min(tempCommonMaxModuleSpeed, modConst.SpeedAt12VoltsMps); //@TODO this uses the 12v nominal speed because a direct max speed is not yet exposed
    }
    commonMaxModuleSpeed = tempCommonMaxModuleSpeed;
    if (Utils.isSimulation()) {
      startSimThread();
    }
    configPathPlanner();
  }
  
  private void telemetryCallbackWrapperFunction(SwerveDriveState state) {
    this.log("Pose", state.Pose);
    this.log("Speeds", state.speeds);
    this.log("ModuleStates", state.ModuleStates);
    this.log("TargetModuleStates", state.ModuleTargets);
    this.log("SuccessfulDAQs", state.SuccessfulDaqs);
    this.log("FailedDAQs", state.FailedDaqs);
    this.log("OdometryPeriod", state.OdometryPeriod);
    if (userTelemetryCallback != null) {
      userTelemetryCallback.accept(state);
    }
  }
  
      /**
     * Register the specified lambda to be executed whenever our SwerveDriveState function
     * is updated in our odometry thread.
     * <p>
     * It is imperative that this function is cheap, as it will be executed along with
     * the odometry call, and if this takes a long time, it may negatively impact
     * the odometry of this stack.
     * <p>
     * This can also be used for logging data if the function performs logging instead of telemetry
     *
     * @param telemetryFunction Function to call for telemetry or logging
     */
    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
      try {
          m_stateLock.writeLock().lock();
          userTelemetryCallback = telemetryFunction;
      } finally {
          m_stateLock.writeLock().unlock();
      }
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    return  m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public ChassisSpeeds getCurrentFieldRelitiveChassisSpeeds() {
    return  BreakerMath.fromRobotRelativeSpeeds(getState().speeds, getState().Pose.getRotation());
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void configPathPlanner() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
        driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }
    AutoBuilder.configureHolonomic(
      ()->this.getState().Pose, // Supplier of current robot pose
      this::seedFieldRelative,  // Consumer for seeding pose against auto
      this::getCurrentChassisSpeeds,
      (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
      new HolonomicPathFollowerConfig(constants.pathFollowerTranslationPID,
                                      constants.pathFollowerRotationPID,
                                      commonMaxModuleSpeed,
                                      driveBaseRadius,
                                      constants.pathFollowerReplanningConfig),
      () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
      this); // Subsystem for requirements
  }

  private void startSimThread() {
        lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        simNotifier.startPeriodic(1.0/constants.simUpdateFrequency);
  }

  public Rotation2d getOperatorForwardDirection() {
    return m_operatorForwardDirection;
  }

  @Override
  public void periodic() {
   /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
          DriverStation.getAlliance().ifPresent((allianceColor) -> {
              this.setOperatorPerspectiveForward(
                      allianceColor == Alliance.Red ? constants.redAlliancePerspectiveRotation
                              : constants.redAlliancePerspectiveRotation);
              hasAppliedOperatorPerspective = true;
        });
      }
  }

  public static class BreakerSwerveDrivetrainConstants extends SwerveDrivetrainConstants {
    public PIDConstants pathFollowerTranslationPID = new PIDConstants(10, 0, 0);
    public PIDConstants pathFollowerRotationPID = new PIDConstants(10, 0, 0);
    public ReplanningConfig pathFollowerReplanningConfig = new ReplanningConfig();
    public double odometryUpdateFrequency = 250;
    public double simUpdateFrequency = 200;
    public Rotation2d blueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    public Rotation2d redAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);

    public BreakerSwerveDrivetrainConstants() {
      super();
    }

    @Override
    public BreakerSwerveDrivetrainConstants withCANbusName(String name) {
      this.CANbusName = name;
      return this;
    }

    @Override
    public BreakerSwerveDrivetrainConstants withPigeon2Configs(Pigeon2Configuration configs) {
      this.Pigeon2Configs = configs;
      return this;
    }

    @Override
    public BreakerSwerveDrivetrainConstants withPigeon2Id(int id) {
      this.Pigeon2Id = id;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withPathFollowerTranslationPID(PIDConstants pidConstants) {
      pathFollowerTranslationPID = pidConstants;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withPathFollowerRotationPID(PIDConstants pidConstants) {
      pathFollowerRotationPID = pidConstants;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withPathFollowerReplaningConfig(ReplanningConfig replanningConfig) {
      pathFollowerReplanningConfig = replanningConfig;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withOdometryUpdateFrequency(double frequencyHz) {
      odometryUpdateFrequency = frequencyHz;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withSimUpdateFrequency(double frequencyHz) {
      simUpdateFrequency = frequencyHz;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withBlueAlliancePerspectiveRotation(Rotation2d perspectiveRotation) {
      blueAlliancePerspectiveRotation = perspectiveRotation;
      return this;
    }

    public BreakerSwerveDrivetrainConstants withRedAlliancePerspectiveRotation(Rotation2d perspectiveRotation) {
      redAlliancePerspectiveRotation = perspectiveRotation;
      return this;
    }

  } 
}
