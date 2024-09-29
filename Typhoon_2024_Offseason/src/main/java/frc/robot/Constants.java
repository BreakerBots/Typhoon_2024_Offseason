package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import frc.robot.BreakerLib.swerve.BreakerSwerveDrivetrain.BreakerSwerveDrivetrainConstants;
import frc.robot.BreakerLib.util.MechanismRatio;
import frc.robot.BreakerLib.util.math.BreakerUnits;
import frc.robot.subsystems.shooter.Shooter.ShooterState;

public class Constants {


  public static class GeneralConstants {
    public static final String DRIVE_CANIVORE_NAME = "drive_canivore";
  }

  public static class FieldConstants {
        public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        public static final double FIELD_WIDTH = APRILTAG_FIELD_LAYOUT.getFieldWidth();
        public static final double FIELD_LENGTH = APRILTAG_FIELD_LAYOUT.getFieldLength();
        public static final Translation3d BLUE_SPEAKER_AIM_POINT = new Translation3d(-0.038099999999999995, 5.547867999999999, 1.4511020000000001).plus(new Translation3d(edu.wpi.first.math.util.Units.inchesToMeters(-10.0), edu.wpi.first.math.util.Units.inchesToMeters(0), edu.wpi.first.math.util.Units.inchesToMeters(30.322)));
        public static final Translation3d BLUE_PASS_AIM_POINT = new Translation3d();
  }

  public static class BatteryTrackerConstants {
        public static final String LIMELIGHT_NAME = "BatteryLL";
        public static final String BATTERY_NAME_FILE_PATH = "/home/lvuser/prev_battery.txt";
        public static final String DEFAULT_BATTERY_NAME = "UNKNOWN";
  }

  public static class ApriltagVisionConstants {

        public static final String FRONT_LEFT_CAMERA_NAME = "";
        public static final String FRONT_RIGHT_CAMERA_NAME = "";
        public static final String BACK_LEFT_CAMERA_NAME = "";
        public static final String BACK_RIGHT_CAMERA_NAME = "";
        public static final List<String> CAMERA_NAMES = Arrays.asList(FRONT_LEFT_CAMERA_NAME, FRONT_RIGHT_CAMERA_NAME, BACK_LEFT_CAMERA_NAME, BACK_RIGHT_CAMERA_NAME);

        public static final SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties();
        
        public static final Transform3d FRONT_LEFT_CAMERA_TRANSFORM = new Transform3d();
        public static final Transform3d FRONT_RIGHT_CAMERA_TRANSFORM = new Transform3d();
        public static final Transform3d BACK_LEFT_CAMERA_TRANSFORM = new Transform3d();
        public static final Transform3d BACK_RIGHT_CAMERA_TRANSFORM = new Transform3d();

        public static final Vector<N3> BASE_SINGLE_TAG_DEVS = VecBuilder.fill(8, 8, 8);
        public static final Vector<N3> BASE_MULTI_TAG_DEVS = VecBuilder.fill(0.5, 0.5, 1);
        public static final Measure<Distance> MAX_SINGLE_TAG_DETECTION_DISTANCE = Units.Meters.of(3.5);
        public static final Measure<Distance> MAX_MULTI_TAG_DETECTION_DISTANCE = Units.Meters.of(6.5);
        public static final double LINEAR_VEL_SCAILAR = 0.0;
        public static final double ANGULAR_VEL_SCAILAR = 0.32;
        public static final double DISTANCE_INVERSE_SCAILAR = 10.0;
        

        
  }

  public static class DriverConstants {
        public static final double TRANSLATIONAL_DEADBAND = 0.05;
        public static final double ROTATIONAL_DEADBAND = 0.05;
  }

  public static class AmpBarConstants {
        public static final Rotation2d EXTENDED_ANGLE_THRESHOLD = Rotation2d.fromRotations(0.39);
        public static final Rotation2d RETRACTED_ANGLE_THRESHOLD = Rotation2d.fromRotations(0.1);
        public static final Rotation2d ENCODER_OFFSET = Rotation2d.fromRotations(-0.523681640625);
      
        public static final int AMP_BAR_ENCODER_ID = 35;
        public static final int AMP_BAR_MOTOR_ID = 60;
    }

    public static class HopperConstants {
        public static final int HOPPER_ID = 33;
    }

  public static class ShooterConstants {
        public static final int LEFT_FLYWHEEL_ID = 30;
        public static final int RIGHT_FLYWHEEL_ID = 31;
        public static final int SHOOTER_PIVOT_ID = 32;
        public static final int PIVOT_ENCODER_ID = 34;

        public static final MechanismRatio FLYWHEEL_RATIO = new MechanismRatio();
        public static final MechanismRatio PIVOT_RATIO = new MechanismRatio(9.0).to(new MechanismRatio(14, 215));
          /**
         * The conversion between shooter tangential velocity and note launch velocity.
         */
        public static final double NOTE_LAUCH_VEL_LOSS_SCAILAR = 4.75;
        public static final Measure<Distance> FLYWHEEL_RADIUS = Units.Inches.of(2.0);
        public static final Translation3d PIVOT_AXEL_TO_FLYWHEEL_TRANS = new Translation3d(-0.0, 0.0, 0.0);
        public static final Translation3d ROBOT_TO_PIVOT_AXEL_TRANS = new Translation3d(0.0, 0.0, 0.0);

        public static final Measure<Velocity<Angle>> MIN_VEL = Units.RotationsPerSecond.of(0.0);
        public static final Measure<Velocity<Angle>> MAX_VEL = Units.RotationsPerSecond.of(100.0);

        public static final Rotation2d STOW_ANGLE =  Rotation2d.fromRotations(0.016);
        public static final Rotation2d MIN_ANGLE =  Rotation2d.fromRotations(0.01);
        public static final Rotation2d MAX_ANGLE =  Rotation2d.fromRotations(0.21);

        public static final Rotation2d INTAKE_HANDOFF_ANGLE = Rotation2d.fromRotations(0.016);
        public static final ShooterState INTAKE_HANDOFF_STATE = new ShooterState(INTAKE_HANDOFF_ANGLE, Units.RadiansPerSecond.of(0.0));

        //control constants
        public static final Measure<Angle> PITCH_SETPOINT_CONTROL_TYPE_SWITCH_EPSILON = Units.Degrees.of(0.002);
        public static final Measure<Angle> MOTION_MAGIC_MIN_DIST = Units.Degrees.of(6.0);
        public static final double PITCH_ENCODER_OFFSET = -0.40625+0.125;

        //feedforward
        public static final double PIVOT_kV = 17.25;
        public static final double PIVOT_kA = 0.18;
        public static final double PIVOT_kS = 0.025;
        public static final double PIVOT_kG = 0.43;
        public static final double PIVOT_kX = 0.0;

        public static final double PIVOT_MOTION_MAGIC_kP = 7.5;
        public static final double PIVOT_MOTION_MAGIC_kI = 0.0;
        public static final double PIVOT_MOTION_MAGIC_kD = 0.8;

        public static final double PIVOT_POSITION_kP = 9.5;
        public static final double PIVOT_POSITION_kI = 0.0;
        public static final double PIVOT_POSITION_kD = 0.4;

        public static final double SHOOT_ON_THE_MOVE_DRIVER_INPUT_SCAILAR = 0.3;
        public static final double SHOOT_ON_THE_MOVE_DRIVER_INPUT_RATE_LIMIT = 1.0;

        //Tolerences
        public static final Measure<Angle> PIVOT_POS_TOLERENCE = Units.Degrees.of(2.0);
        public static final Measure<Velocity<Angle>> PIVOT_VEL_TOLERENCE = Units.DegreesPerSecond.of(180.0);
        public static final Measure<Velocity<Angle>> FLYWHEEL_VEL_TOLERENCE = Units.RotationsPerSecond.of(4.0);
        public static final Measure<Velocity<Velocity<Angle>>> FLYWHEEL_ACCEL_TOLERENCE = BreakerUnits.RotationsPerSecondPerSecond.of(5.0);

        //Homeing
        public static final Rotation2d HOME_POS = Rotation2d.fromRotations(0.017334);
        public static final double HOME_VOLTAGE = -3.0;
        public static final Measure<Velocity<Angle>> HOME_TRIGGER_VEL_TOL = Units.RotationsPerSecond.of(1e-5);
        public static final double HOME_TRIGGER_TIME = 0.3;
        public static final double HOME_SPIKE_VOLTAGE = -8.0;
        public static final double HOME_SPIKE_TIME = 0.6;
        public static final double HOME_CURRENT_LIMIT = 15.0;
        
  }
  
  public static class IntakeConstants {
      public static final int ROLLER_MOTOR_ID = 40; 
      public static final int PIVOT_LEFT_ID = 41;
      public static final int PIVOT_RIGHT_ID = 42;
      public static final int PIVOT_ENCODER_ID = 43;
      
      public static final double PIVOT_ENCODER_OFFSET = 0.2822265625;
  
      /*0deg is 15 deg behind fully retracted*/
      public static final double PIVIOT_RETRACTED_THRESHOLD = 0.32;//0.32;
      /*0deg is 20 deg behind fully retracted*/
      public static final double PIVIOT_EXTENDED_THRESHOLD = 0.02;//0.008;
  
      public static final double PIVOT_AGAINST_AMP_ANGLE_THRESHOLD = 0.30; // temporary

      public static final Measure<Distance> MAX_SMART_ROLLER_ENABLE_NOTE_DISTANCE = Units.Meters.of(2.5);
      public static final double SMART_ROLLER_MAX_WAIT_FOR_DET = 1.0;
    }

    public static class IntakeConstants2 {
        public static final int ROLLER_MOTOR_ID = 40; 
        public static final int PIVOT_LEFT_ID = 41;
        public static final int PIVOT_RIGHT_ID = 42;
        public static final int PIVOT_ENCODER_ID = 43;
        
        public static final double PIVOT_ENCODER_OFFSET = 0.0;

        public static final MechanismRatio PIVOT_RATIO = new MechanismRatio(18.0);

        public static final Measure<Angle> PIVOT_DEFAULT_TOLERENCE = Units.Degrees.of(2.5);
        
        public static final Rotation2d PIVOT_EXTENDED_ANGLE = Rotation2d.fromRotations(0.0);
        public static final Measure<Angle> PIVOT_EXTENDED_TOLERENCE = Units.Degrees.of(5.0);
        public static final Rotation2d PIVOT_RETRACTED_ANGLE = Rotation2d.fromRotations(0.0);
        public static final Measure<Angle> PIVOT_RETRACTED_TOLERENCE = Units.Degrees.of(3.5);
        public static final Rotation2d PIVOT_AMP_ANGLE = Rotation2d.fromRotations(0.0);
        public static final Measure<Angle> PIVOT_AMP_TOLERENCE = Units.Degrees.of(2.5);
    }

    public static class DriveConstants {
        public static final Measure<Velocity<Distance>> MAXIMUM_TRANSLATIONAL_VELOCITY = Units.MetersPerSecond.of(4.5);
        public static final Measure<Velocity<Angle>> MAXIMUM_ROTATIONAL_VELOCITY = Units.RadiansPerSecond.of(9.5);
      // Both sets of gains need to be tuned to your individual robot.
      // The steer motor uses any SwerveModule.SteerRequestType control request with the
      // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
      private static final Slot0Configs steerGains = new Slot0Configs()
          .withKP(100).withKI(0).withKD(0.35)
          .withKS(0).withKV(1.5).withKA(0);
      // When using closed-loop control, the drive motor uses the control
      // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
      private static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(2.04).withKI(0).withKD(0)
          .withKS(0.36).withKV(1.74).withKA(0);

      // The closed-loop output type to use for the steer motors;
      // This affects the PID/FF gains for the steer motors
      private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      // The closed-loop output type to use for the drive motors;
      // This affects the PID/FF gains for the drive motors
      private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

      // The stator current at which the wheels start to slip;
      // This needs to be tuned to your individual robot
      private static final double kSlipCurrentA = 80.0;

      // Initial configs for the drive and steer motors and the CANcoder; these cannot be null.
      // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
      private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
      private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  // Swerve azimuth does not require much torque output, so we can set a relatively low
                  // stator current limit to help avoid brownouts without impacting performance.
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
          );
      private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
      // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
      private static final Pigeon2Configuration pigeonConfigs = null;

      // Theoretical free speed (m/s) at 12v applied output;
      // This needs to be tuned to your individual robot
      public static final double kSpeedAt12VoltsMps = 0;

      // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
      // This may need to be tuned to your individual robot
      private static final double kCoupleRatio = 3.125;

      private static final double kDriveGearRatio = 5.357142857142857;
      private static final double kSteerGearRatio = 21.428571428571427;
      private static final double kWheelRadiusInches = 4.0;

      private static final boolean kSteerMotorReversed = true;

      private static final String kCANbusName = GeneralConstants.DRIVE_CANIVORE_NAME;
      private static final int kPigeonId = 5;


      // These are only used for simulation
      private static final double kSteerInertia = 0.00001;
      private static final double kDriveInertia = 0.001;
      // Simulated voltage necessary to overcome friction
      private static final double kSteerFrictionVoltage = 0.25;
      private static final double kDriveFrictionVoltage = 0.25;

      public static final BreakerSwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new BreakerSwerveDrivetrainConstants()
              .withCANbusName(kCANbusName)
              .withPigeon2Id(kPigeonId)
              .withPigeon2Configs(pigeonConfigs);

      private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withWheelRadius(kWheelRadiusInches)
              .withSlipCurrent(kSlipCurrentA)
              .withSteerMotorGains(steerGains)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(kCoupleRatio)
              .withSteerMotorInverted(kSteerMotorReversed)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withCANcoderInitialConfigs(cancoderInitialConfigs);

      private static final boolean kInvertLeftSide = true;
      private static final boolean kInvertRightSide = false;

      // Front Left
      private static final int kFrontLeftDriveMotorId = 10;
      private static final int kFrontLeftSteerMotorId = 11;
      private static final int kFrontLeftEncoderId = 20;
      private static final double kFrontLeftEncoderOffset = -0.223388671875;
      private static final Translation2d kFrontLeftModulePosition = new Translation2d(0.314325, 0.314325);

      // Front Right
      private static final int kFrontRightDriveMotorId = 12;
      private static final int kFrontRightSteerMotorId = 13;
      private static final int kFrontRightEncoderId = 21;
      private static final double kFrontRightEncoderOffset = -0.06591796875;
      private static final Translation2d kFrontRightModulePosition = new Translation2d(0.314325, -0.314325);


      // Back Left
      private static final int kBackLeftDriveMotorId = 14;
      private static final int kBackLeftSteerMotorId = 15;
      private static final int kBackLeftEncoderId = 22;
      private static final double kBackLeftEncoderOffset = -0.325439453125;
      private static final Translation2d kBackLeftModulePosition = new Translation2d(-0.314325, 0.314325);

      // Back Right
      private static final int kBackRightDriveMotorId = 16;
      private static final int kBackRightSteerMotorId = 17;
      private static final int kBackRightEncoderId = 23;
      private static final double kBackRightEncoderOffset = 0.412353515625;
      private static final Translation2d kBackRightModulePosition = new Translation2d(-0.314325, -0.314325);


      public static final SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, kFrontLeftModulePosition.getX(), kFrontLeftModulePosition.getY(),  kInvertLeftSide);
      public static final SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, kFrontRightModulePosition.getX(), kFrontRightModulePosition.getY(),  kInvertRightSide);
      public static final SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, kBackLeftModulePosition.getX(), kBackLeftModulePosition.getY(),  kInvertLeftSide);
      public static final SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, kBackRightModulePosition.getX(), kBackRightModulePosition.getY(),  kInvertRightSide);
    }
    
}
