// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.BreakerLib.util.loging.BreakerLog;
import frc.robot.subsystems.LED;

import static frc.robot.Constants.BatteryTrackerConstants.*;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.sql.Driver;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;

import com.fasterxml.jackson.databind.ser.std.StdArraySerializers.FloatArraySerializer;

import dev.doglog.AdvantageKitCompatibleLogger;

/** Add your docs here. */
public class BatteryTracker extends SubsystemBase {
    private Optional<String> batteryName = Optional.empty();
    private boolean batteryNameChecked = false;
    private boolean batteryNameWritten = false;
    private boolean sameBatteryFlag = false;
    private LED led;
    public BatteryTracker() {}

    public Optional<String> getBatteryName() {
        return batteryName;
    }

    public Optional<Boolean> hasBatteryBeenChanged() {
        return batteryNameChecked ? Optional.of(sameBatteryFlag) : Optional.empty();
    }

    private void registerCapture(String name) {
        CommandScheduler.getInstance().schedule(Commands.sequence(
            Commands.runOnce(() -> AdvantageKitCompatibleLogger.recordMetadata("BatteryName", name), this),
            Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceBlink(LIMELIGHT_NAME), this),
            Commands.waitSeconds(3.0),
            Commands.runOnce(() -> LimelightHelpers.setLEDMode_ForceOff(LIMELIGHT_NAME), this)
        ).ignoringDisable( true));
    }

    @Override
    public void periodic() {
        if (batteryName.isEmpty()) {
            LimelightHelpers.setLEDMode_ForceOn(LIMELIGHT_NAME);
            if (LimelightHelpers.getTargetCount(LIMELIGHT_NAME) > 0) {
                batteryName =  Optional.of(LimelightHelpers.getRawBarcodeData(LIMELIGHT_NAME)[0]);
                registerCapture(batteryName.get());
            } else if (RobotState.isEnabled()) {
                BreakerLog.logFault("FAILED TO CAPTURE BATTERY NAME");
                registerCapture(DEFAULT_BATTERY_NAME);
            }
        }

        BreakerLog.log("Battery/Name", batteryName.orElse(DEFAULT_BATTERY_NAME));
        if (Robot.isReal() && batteryName.isPresent()) {
            // Check for battery alert
            if (!batteryNameChecked) {
                batteryNameChecked = true;
                File file = new File(BATTERY_NAME_FILE_PATH);
                if (file.exists()) {
                    // Read previous battery name
                    String previousBatteryName = "";
                    try {
                        previousBatteryName =
                            new String(Files.readAllBytes(Paths.get(BATTERY_NAME_FILE_PATH)), StandardCharsets.UTF_8);
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                    if (previousBatteryName.equals(batteryName.get())) {
                        // Same battery, set alert
                        sameBatteryFlag = true;
                        led.setErroring();
                        DriverStation.reportWarning("BATTERY NOT CHANGED SINCE LAST MATCH", false);
                    } else {
                        // New battery, delete file
                        file.delete();
                    }
                }
            }

            // Write battery name if connected to FMS
            if (!batteryNameWritten && DriverStation.isFMSAttached()) {
                batteryNameWritten = true;
                try {
                    FileWriter fileWriter = new FileWriter(BATTERY_NAME_FILE_PATH);
                    fileWriter.write(batteryName.get());
                    fileWriter.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
