// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.BreakerLib.sensors.BreakerBeamBreak;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  private WPI_TalonSRX hopperMotor;
  private HopperState state;
  public Hopper() {

  }

  public static enum HopperState {
    INTAKE_TO_HOPPER(-1.0, true),
    HOPPER_TO_FLYWHEEL(-1.0, false),
    HOPPER_TO_INTAKE(1.0, false),
    NEUTRAL(0.0, true);
    private double dutyCycle;
    public boolean stopOnBeamBreakTrigger;
    private HopperState(double dutyCycle, boolean stopOnBeamBreakTrigger) {
      this.dutyCycle = dutyCycle;
      this.stopOnBeamBreakTrigger = stopOnBeamBreakTrigger;
    }

    public double getDutyCycle() {
        return dutyCycle;
    }

    public boolean getStopOnBeamBreakTrigger() {
        return stopOnBeamBreakTrigger;
    }
  }

  public boolean hasNote() {
    return hopperMotor.isRevLimitSwitchClosed() == 0;
  }

  public HopperState getState() {
    return state;
  }

  public Command setStateCommand(HopperState state) {
    return new InstantCommand(() -> setState(state), this);
  }

  private void setState(HopperState state) {
    this.state = state;
    hopperMotor.set(state.getDutyCycle());
    hopperMotor.overrideLimitSwitchesEnable(state.getStopOnBeamBreakTrigger());
  }

  @Override
  public void periodic() {
    
  }
}
