// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.devices.BreakerRevBlinkin;
import frc.robot.BreakerLib.devices.BreakerRevBlinkin.FixedPalettePattern;
import frc.robot.BreakerLib.devices.BreakerRevBlinkin.SolidColor;


public class LED extends SubsystemBase {
  public static class Patterns {
    private Optional<FixedPalettePattern> fixedPalettePattern = Optional.empty();
    private Optional<SolidColor> solidColor = Optional.empty();

    public Patterns(FixedPalettePattern pattern) {
      fixedPalettePattern = Optional.of(pattern);
    }

    public Patterns(SolidColor color) {
      solidColor = Optional.of(color);
    }

    public boolean isFixedPalettePattern() {
      return fixedPalettePattern.isPresent();
    }

    public boolean isSolidColor() {
      return solidColor.isPresent();
    }
    
    public FixedPalettePattern getFixedPalettePattern() {
      return fixedPalettePattern.get();
    }

    public SolidColor getSolidColor() {
      return solidColor.get();
    }
     
  }
  public enum LEDState {
    INTAKING_FOR_SHOOTER(SolidColor.SKY_BLUE),
    INTAKING_FOR_AMP(SolidColor.GOLD),
    EXTAKING(FixedPalettePattern.HEARTBEAT_BLUE),
    
    SHOOTING_TO_SPEAKER(FixedPalettePattern.STROBE_BLUE),
    SCORING_IN_AMP(SolidColor.BLUE_VIOLET),
    PERSUING_NOTE(FixedPalettePattern.RAINBOW_GLITTER),
    
    ENABLED_WITH_NOTE(FixedPalettePattern.STROBE_GOLD),
    ENABLED(FixedPalettePattern.BREATH_BLUE), // should be 'default' state
    
    SPOOLING_FLYWHEEL(FixedPalettePattern.CONFETTI),

    ERROR(FixedPalettePattern.HEARBEAT_RED),
    FLASH_ON_INTAKE(FixedPalettePattern.STROBE_GOLD);

    private Patterns pattern;


    private LEDState(FixedPalettePattern pattern) {
      this.pattern = new Patterns(pattern);
    }

    private LEDState(SolidColor color) {
      this.pattern = new Patterns(color);
    }

    public Patterns getPattern() {
      return pattern;
    }

  }
  
  private final BreakerRevBlinkin blinkin;
  private LEDState currentState;
  private boolean isErrored = false;

  /** Subsystem for managing LED lights on the robot using a state machine. */
  public LED() {
    blinkin = new BreakerRevBlinkin(0);
    currentState = LEDState.ENABLED;
    blinkin.setSolidColor(SolidColor.WHITE);
    updateBlinkin();
  }

  public Command setErroringCommand() {
    return runOnce(() -> setErroring());
  }

  public Command clearErrorCommand(boolean hasNote) {
    return runOnce(() -> clearError(hasNote));
  }

  public void setErroring() {
    setState(LEDState.ERROR);
    isErrored = true;
  }

  public void clearError(boolean hasNote) {
    isErrored = false;
    setRestState(hasNote);
  }

  public Command setStateCommand(LEDState state) {
    return runOnce(() -> setState(state));
  }

  /**
   * Returns to the 'default' state, which is either {@code LEDState.ENABLED} or {@code LEDState.ENABLED_WITH_NOTE}
   * 
   * @return nothing
   */
  public Command returnToRestState(boolean hasNote) {
    return runOnce(() -> {
      setRestState(hasNote);
    });
  }

  public void setRestState(boolean hasNote) {
    if (hasNote) {
        setState(LEDState.ENABLED_WITH_NOTE);
      } else {
        setState(LEDState.ENABLED);
      }
  }

  public void setState(LEDState state) {
    if (isErrored) return; // allow error colors to always take priority

    currentState = state;
    updateBlinkin();
  }

  private void updateBlinkin() {
    Patterns pattern = currentState.getPattern();
    if (pattern.isFixedPalettePattern()) {
      blinkin.setFixedPalettePattern(pattern.getFixedPalettePattern());
    } else {
      blinkin.setSolidColor(pattern.getSolidColor());
    }
  }

  public LEDState getCurrentState() {
    return currentState;
  }
  @Override
  public void periodic() {
    
    //BreakerDashboard.getMainTab().addBoolean("HAS NOTE", this::hasNote);
    // This method will be called once per scheduler run
  }
}
