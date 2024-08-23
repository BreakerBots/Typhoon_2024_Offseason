// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.loging;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.physics.ChassisAccels;

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
    
    public static void logMetadata(Metadata metadata) {

    }



    public record Metadata() {
    }
}
