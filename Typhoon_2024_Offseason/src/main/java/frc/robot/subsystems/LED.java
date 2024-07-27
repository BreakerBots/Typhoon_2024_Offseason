// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  private CANdle candle;
  private static final int LED_COUNT = 300;
  public LED() {
    candle = new CANdle(0);
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.GRB;
    config.brightnessScalar = 1.0;
    config.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(config);
    
  }

  private static class ColorRefrence {
    public static final Color8Bit ORANGE = new Color8Bit(Color.kOrange);
    public static final Color8Bit YELLOW = new Color8Bit(Color.kYellow);
  }

  public static enum AnimationType {
    DISABLED(new RgbFadeAnimation(1, 0.5, LED_COUNT)),
    ENABLED(new RainbowAnimation(1, 1, LED_COUNT)),
    ENABLED_WITH_NOTE_IN_HOPPER(new StrobeAnimation(ColorRefrence.ORANGE.red, ColorRefrence.ORANGE.green, ColorRefrence.ORANGE.blue, 0, 0.5, LED_COUNT, 0)),
    ENABLED_WITH_NOTE_IN_INTAKE(new StrobeAnimation(ColorRefrence.YELLOW.red, ColorRefrence.YELLOW.green, ColorRefrence.YELLOW.blue, 0, 0.5, LED_COUNT, 0))
    
    ;
    private AnimationType(Animation anim) {

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
