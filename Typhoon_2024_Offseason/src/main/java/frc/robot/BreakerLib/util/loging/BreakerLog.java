// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.loging;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.AdvantageKitCompatibleLogger;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import frc.robot.BuildConstants;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.physics.ChassisAccels;
import frc.robot.BreakerLib.util.BreakerLibVersion;

/** Add your docs here. */
public class BreakerLog extends DogLog {

    
    public static void log(String key, BreakerVector2 value) {
        log(key + "/x", value.getX());
        log(key + "/y", value.getY());
        log(key + "/angle", value.getAngle());
    }

    public static void log(String key, BreakerVector3 value) {
        log(key + "/x", value.getX());
        log(key + "/y", value.getY());
        log(key + "/z", value.getZ());
        log(key + "/angle", value.getAngle());
    }

    public static void log(String key, ChassisAccels value) {
        log(key + "/x", value.getX());
        log(key + "/y", value.getY());
        log(key + "/alpha", value.getAlpha());
    }

    public static void log(String key, TalonFX value) {
        log(key + "/StatorCurrent", value.getStatorCurrent().getValue());
        log(key + "/Position", value.getPosition().getValue());
        log(key + "/Velocity", value.getVelocity().getValue());
    }
    
    public static void logMetadata(String key, String value) {
        AdvantageKitCompatibleLogger.recordMetadata(key, value);
    }

    public static void logMetadata(Metadata metadata) {
        logMetadata("RobotName", metadata.robotName);
        logMetadata("ProjectYear", Integer.toString(metadata.year));
        logMetadata("Authors", metadata.authors);
        logMetadata("WPILibVersion", metadata.wpiLibVersion);
        logMetadata("BreakerLibVersion", metadata.breakerLibVersion);
        logMetadata("MavenName", metadata.mavenName);
        logMetadata("GitRevision", Integer.toString(metadata.gitRevision));
        logMetadata("GitSHA", metadata.gitSHA);
        logMetadata("GitDate", metadata.gitDate);
        logMetadata("GitBranch", metadata.gitBranch);
        logMetadata("BuildDate", metadata.buildDate);
        logMetadata("Dirty", Integer.toString(metadata.dirty));
    }


    public record Metadata(
        String robotName,
        int year,
        String authors,
        String wpiLibVersion,
        String breakerLibVersion,
        String mavenName,
        int gitRevision,
        String gitSHA,
        String gitDate,
        String gitBranch,
        String buildDate,
        int dirty
    ) {
        public Metadata(String robotName, int year, String authors) {
            this(robotName, year, authors, WPILibVersion.Version, BreakerLibVersion.version, BuildConstants.MAVEN_NAME, BuildConstants.GIT_REVISION, BuildConstants.GIT_SHA, BuildConstants.GIT_DATE, BuildConstants.GIT_BRANCH, BuildConstants.BUILD_DATE, BuildConstants.DIRTY);
        }
    }
}
