// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static java.lang.Math.atan;
import static java.lang.Math.log;
import static java.lang.Math.negateExact;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake2;
import frc.robot.subsystems.shooter.Shooter.ShooterState;
import frc.robot.subsystems.shooter.Shooter.SmartSpoolConfig;

import static frc.robot.Constants.ShooterConstants.*;

/** Add your docs here. */
public class ShooterTarget {
    private static final ShootOnTheMoveType SHOOT_ON_THE_MOVE_TYPE = ShootOnTheMoveType.VECTOR;
    public static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();
    public static final Measure<Angle> YAW_AIM_TOLERENCE = Units.Degrees.of(2.5);
    private Translation3d blueTargetPoint;
    private final Drivetrain drivetrain;
    private final Shooter shooter;
    private final Hopper hopper;
    private SwerveRequest.FieldCentricFacingAngle driveRequest = new SwerveRequest.FieldCentricFacingAngle();
    private Function<Double, Double> angleFunction;
    private Function<Double, Double> speedFunction;
    public SmartSpoolConfig smartSpoolConfig;
    public ShooterTarget(Drivetrain drivetrain, Shooter shooter, Hopper hopper, Translation3d blueTargetPoint ) {
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.hopper = hopper;
        this.blueTargetPoint = blueTargetPoint;
        driveRequest.ForwardReference = ForwardReference.RedAlliance;
        driveRequest.DriveRequestType = DriveRequestType.Velocity;
        driveRequest.HeadingController.setPID(2.5, 0.0, 0.1);
        driveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command runSmartSpool(Intake intake) {
        return shooter.runShooter(() -> {
            double dist = drivetrain.getState().Pose.getTranslation().getDistance(getTargetPosition().toTranslation2d());
            ShooterState idealState = getStationaryShooterState(dist);
            Rotation2d pitchAngle = STOW_ANGLE;
            Measure<Velocity<Angle>> flywheelVel = Units.RadiansPerSecond.of(0.0);
            if (!intake.hasNote() && hopper.hasNote()) {
                if (dist <= smartSpoolConfig.flywheelSpoolDistance().in(Units.Meters)) {
                    flywheelVel = idealState.flywheelVel();
                }
                if (dist <= smartSpoolConfig.pitchTrackDistance().in(Units.Meters)) {
                    pitchAngle = idealState.pitchAngle();
                }
            }
            return new ShooterState(pitchAngle, flywheelVel);}, intake, hopper);
    }

    public static Command shoot(Supplier<ShooterState> stateSupplier, BooleanSupplier shootCondition, Shooter shooter, Hopper hopper) {
        return Commands.waitUntil(() -> shooter.atSetpoint() && shootCondition.getAsBoolean())
        .andThen(hopper.feedFlywheel())
        .deadlineWith(shooter.runShooter(stateSupplier))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming); //enshures the shooting process will
    }

    public static Command shoot(Supplier<ShooterState> stateSupplier, Shooter shooter, Hopper hopper) {
        return shoot(stateSupplier, () -> true, shooter, hopper);
    }

    public Command shootWhileMoveingAuton() {
        return shoot(
            () -> getMovingFireingSolution().shooterState(), 
            () -> atYaw(getMovingFireingSolution(Units.Seconds.of(0.2)).robotYaw()), 
            shooter, 
            hopper)
            .beforeStarting(
                Commands.runOnce(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(getMovingFireingSolution().robotYaw())))
            )
            .andThen(
                Commands.runOnce(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.empty()))
            );
    }

    public Command shootWhileMoveing(BreakerInputStream x, BreakerInputStream y) {
        var scaleX = x.scale(SHOOT_ON_THE_MOVE_DRIVER_INPUT_SCAILAR).rateLimit(SHOOT_ON_THE_MOVE_DRIVER_INPUT_RATE_LIMIT);
        var scaleY = y.scale(SHOOT_ON_THE_MOVE_DRIVER_INPUT_SCAILAR).rateLimit(SHOOT_ON_THE_MOVE_DRIVER_INPUT_RATE_LIMIT);
        return shoot(
            () -> getMovingFireingSolution().shooterState(), 
            () -> atYaw(getMovingFireingSolution(Units.Seconds.of(0.2)).robotYaw()), 
            shooter, 
            hopper)
            .deadlineWith(
                drivetrain.applyRequest(
                    () -> {
                        var driverVec = new BreakerVector2(scaleX.get(), scaleY.get()).rotateBy(drivetrain.getOperatorForwardDirection());
                        return driveRequest.withVelocityX(driverVec.getX()).withVelocityY(driverVec.getY()).withTargetDirection(getMovingFireingSolution().robotYaw());
                    }
                )
            );
    }

    public boolean atYaw(Rotation2d yaw) {
        double tolerance = YAW_AIM_TOLERENCE.in(Units.Radians) * (1 - yaw.getSin());
        Rotation2d diff = drivetrain.getState().Pose.getRotation().minus(yaw);
        return Math.abs(atan(diff.getTan())) < tolerance;
    }

    public Translation3d getTargetPosition() {
        Optional<Alliance> allainceOpt =  DriverStation.getAlliance();
        if (allainceOpt.isPresent()) {
            if (allainceOpt.get() == Alliance.Red) {
                final double halfFieldWidth = 16.541/2.0;//Constants.FieldConstants.FIELD_WIDTH
                double deltaX = halfFieldWidth - -0.29209999; //blueTargetPoint.getX();
                double redX = halfFieldWidth + deltaX;
                return new Translation3d(redX, blueTargetPoint.getY(), blueTargetPoint.getZ());
            }
        }
        return blueTargetPoint; 
    }


    private FireingSolution getMovingFireingSolution(Measure<Time> predictionTime) {
        return getMovingFireingSolution(drivetrain.getState().Pose.getTranslation(), drivetrain.getState().speeds, predictionTime);
    }
    
    private FireingSolution getMovingFireingSolution() {
       return getMovingFireingSolution(drivetrain.getState().Pose.getTranslation(), drivetrain.getState().speeds);
    }

    private FireingSolution getMovingFireingSolution(Translation2d robotTranslation, ChassisSpeeds robotSpeeds, Measure<Time> predictionTime) {
        Translation2d predictedTrans = robotTranslation.plus(new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond).times(predictionTime.in(Units.Seconds)));
        return getMovingFireingSolution(predictedTrans, robotSpeeds);
    }
    
    private FireingSolution getMovingFireingSolution(Translation2d robotTranslation, ChassisSpeeds robotSpeeds) {
        // BreakerVector2 robotSpeedVec = new BreakerVector2(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        // BreakerVector2 clampedVec = robotSpeedVec.clampMagnitude(0.0, Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond) * (SHOOT_ON_THE_MOVE_DRIVER_INPUT_SCAILAR + 0.05));
        // ChassisSpeeds clampedSpeeds = new ChassisSpeeds(clampedVec.getX(), clampedVec.getY(), 0.0);
        if (SHOOT_ON_THE_MOVE_TYPE == ShootOnTheMoveType.PROJECTILE) {
            return getMovingFireingSolutionWithProjectileMath(robotTranslation, robotSpeeds);
        } 
        return getMoveingFireingSolutionWithVectorMath(robotTranslation, robotSpeeds);
    }

    private FireingSolution getMoveingFireingSolutionWithVectorMath(Translation2d robotTranslation, ChassisSpeeds robotSpeeds) {
        FireingSolution stationarySolution = getStationaryFireingSolution(robotTranslation);
        BreakerVector3 stationaryLaunchNoteVelVec = stationarySolution.toNoteVelocity();
        BreakerVector3 corectedVec = stationaryLaunchNoteVelVec.minus(new BreakerVector3(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond, 0.0));
        return FireingSolution.fromNoteVelocity(corectedVec);
    }

    private FireingSolution getMovingFireingSolutionWithProjectileMath(Translation2d robotTranslation, ChassisSpeeds robotSpeeds) {
        FireingSolution stationarySolution = getStationaryFireingSolution(robotTranslation);
        Translation3d launchPoint = getFlywheelTranslationInFieldSpace(new Pose2d(robotTranslation, stationarySolution.robotYaw()), stationarySolution.shooterState.pitchAngle());
        double dist = getTargetPosition().toTranslation2d().getDistance(launchPoint.toTranslation2d());
        BreakerVector2 noteVelVec2d = stationarySolution.toNoteVelocity().toBreakerVector2();
        double time = dist / noteVelVec2d.getMagnitude();
        Translation2d corectedRobotPos = robotTranslation.plus(new Translation2d(robotSpeeds.vxMetersPerSecond * time, robotSpeeds.vyMetersPerSecond * time));
        return getStationaryFireingSolution(corectedRobotPos);
    }

    public Translation3d getFlywheelTranslationInRobotSpace(Rotation2d angle) {
        Translation3d flywheelInAxelSpace = PIVOT_AXEL_TO_FLYWHEEL_TRANS.rotateBy(new Rotation3d(0.0, -Math.PI + angle.getRadians(), 0.0));
        return ROBOT_TO_PIVOT_AXEL_TRANS.plus(flywheelInAxelSpace);
    }

    private Translation3d getFlywheelTranslationInFieldSpace(Pose3d robotPose, Rotation2d angle) {
        Translation3d robotSpaceTrans = getFlywheelTranslationInRobotSpace(angle);
        return robotSpaceTrans.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
    }

    private Translation3d getFlywheelTranslationInFieldSpace(Pose2d robotPose, Rotation2d angle) {
        return getFlywheelTranslationInFieldSpace(new Pose3d(robotPose), angle);
    }

    private FireingSolution getStationaryFireingSolution(Translation2d robotTranslation) {
        Translation2d targetPos = getTargetPosition().toTranslation2d();
        double distance = targetPos.getDistance(robotTranslation);
        Rotation2d angle = robotTranslation.minus(targetPos).getAngle();
        return new FireingSolution(angle, getStationaryShooterState(distance));
    }


    private ShooterState getStationaryShooterState(double distanceMeters) {
        return new ShooterState(Rotation2d.fromRadians(angleFunction.apply(distanceMeters)), Units.RadiansPerSecond.of(speedFunction.apply(distanceMeters)));
    }

    public record FireingSolution(Rotation2d robotYaw, ShooterState shooterState) {
        public static FireingSolution fromNoteVelocity(BreakerVector3 noteVelocityVector) {
            Rotation3d vecRot = noteVelocityVector.getAngle();
            double yaw = vecRot.getZ();
            double rawPitch = vecRot.getY();
            Rotation2d pitch = Rotation2d.fromRadians(rawPitch).plus(Rotation2d.fromDegrees(180));
            Measure<Velocity<Angle>> flywheelAngVel = Units.RadiansPerSecond.of((noteVelocityVector.getMagnitude() / FLYWHEEL_RADIUS.in(Units.Meters)) * NOTE_LAUCH_VEL_LOSS_SCAILAR);
            return new FireingSolution(Rotation2d.fromRadians(yaw), new ShooterState(pitch, flywheelAngVel));
        }
        public BreakerVector3 toNoteVelocity() {
            double velMPS = (shooterState.flywheelVel().in(Units.RadiansPerSecond) * FLYWHEEL_RADIUS.in(Units.Meters)) / NOTE_LAUCH_VEL_LOSS_SCAILAR;
            return new BreakerVector3(velMPS, new Rotation3d(0.0, shooterState.pitchAngle().minus(Rotation2d.fromDegrees(180)).getRadians(), robotYaw.getRadians()));
        }
    }

    public static enum ShootOnTheMoveType {
        VECTOR,
        PROJECTILE
    }
}
