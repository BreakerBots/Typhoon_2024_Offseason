// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.PIVOT_AGAINST_AMP_ANGLE_THRESHOLD;
import static frc.robot.Constants.IntakeConstants.PIVIOT_EXTENDED_THRESHOLD;
import static frc.robot.Constants.IntakeConstants.PIVIOT_RETRACTED_THRESHOLD;
import static frc.robot.Constants.IntakeConstants.PIVOT_ENCODER_ID;
import static frc.robot.Constants.IntakeConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.IntakeConstants.PIVOT_LEFT_ID;
import static frc.robot.Constants.IntakeConstants.PIVOT_RIGHT_ID;
import static frc.robot.Constants.IntakeConstants.ROLLER_MOTOR_ID;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.cscore.VideoProperty;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.BreakerLib.sensors.BreakerBeamBreak;
import frc.robot.BreakerLib.util.factory.BreakerCANCoderFactory;
import frc.robot.BreakerLib.util.loging.BreakerLog;
import frc.robot.subsystems.Intake2.IntakeRollerState;
import frc.robot.subsystems.Intake2.IntakeState.IntakeSetpoint;
import frc.robot.subsystems.vision.ZED;
import frc.robot.subsystems.vision.ZED.TrackedObject;
import frc.robot.BreakerLib.driverstation.gamepad.BreakerGamepadTimedRumbleCommand;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.IntakeConstants2.*;

public class Intake extends SubsystemBase {
  private TalonFX rollerMotor;  
  private TalonFX pivotLeft, pivotRight;
  private IntakeState targetState;
  private CANcoder piviotEncoder;
  private DutyCycleOut piviotDutyCycleControlRequest;
  private Follower pivotFollowerRequest;
  private BreakerBeamBreak beamBreak;

  private boolean isPivotAmpCurrentLimited = false;
  private final CurrentLimitsConfigs normalCurrentPivotConfig;
  private final CurrentLimitsConfigs ampCurrentPivotConfig = new CurrentLimitsConfigs();
  private Supplier<Double> intakePosSupplier;
  
 

  /** Creates a new Intake. */
  public Intake() {
    rollerMotor = new TalonFX(ROLLER_MOTOR_ID);
    pivotLeft = new TalonFX(PIVOT_LEFT_ID);
    pivotRight = new TalonFX(PIVOT_RIGHT_ID);
    piviotEncoder = new CANcoder(PIVOT_ENCODER_ID);

    BreakerCANCoderFactory.configExistingCANCoder(piviotEncoder, AbsoluteSensorRangeValue.Signed_PlusMinusHalf, PIVOT_ENCODER_OFFSET, SensorDirectionValue.CounterClockwise_Positive);
    
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.CurrentLimits.SupplyCurrentLimit = 30;
    pivotConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.5;
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    pivotConfig.Feedback.FeedbackRemoteSensorID = piviotEncoder.getDeviceID();
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    pivotConfig.Feedback.RotorToSensorRatio = 8.4375;
    pivotConfig.Feedback.SensorToMechanismRatio = 1.0;

    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =  PIVIOT_RETRACTED_THRESHOLD;
    pivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PIVIOT_EXTENDED_THRESHOLD;
    pivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;


    pivotLeft.getConfigurator().apply(pivotConfig);
    pivotRight.getConfigurator().apply(pivotConfig);

    normalCurrentPivotConfig = pivotConfig.CurrentLimits;

    ampCurrentPivotConfig.SupplyCurrentLimit = 5;
    ampCurrentPivotConfig.SupplyTimeThreshold = 0.0;
    ampCurrentPivotConfig.SupplyCurrentLimitEnable = true;

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = 60;
    rollerConfig.CurrentLimits.SupplyCurrentThreshold = 180;
    rollerConfig.CurrentLimits.SupplyTimeThreshold = 0.5;

    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerMotor.getConfigurator().apply(rollerConfig);

    piviotDutyCycleControlRequest = new DutyCycleOut(0.0);
    pivotFollowerRequest = new Follower(PIVOT_LEFT_ID, true);

    beamBreak = new BreakerBeamBreak(3, false);

    intakePosSupplier = piviotEncoder.getAbsolutePosition().asSupplier();
  }

  public Command setStateCommand(IntakeState stateToSet, boolean waitForSuccess) {
    return new FunctionalCommand(() -> {setState(stateToSet);}, ()-> {}, (Boolean interrupted) -> {}, () -> {return !waitForSuccess||isAtTargetState();}, this);
  }

  public boolean hasNote() {
    return beamBreak.isBroken();
  }

  public IntakeState getState() {
    return targetState;
  }

  public boolean isAtTargetState() {
    return switch(targetState.getPivotState()) {
      case EXTENDED -> isExtendLimitTriggered();
      case RETRACTED -> isRetractLimitTriggered();
      case AMP -> isAmpLimitTriggered();
      default -> true;
    };
  }

  void setState(IntakeState stateToSet) {
    targetState = stateToSet;
  }

  public static enum IntakeState {
    EXTENDED_EXTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.EXTAKEING),
    EXTENDED_INTAKEING(IntakePivotState.EXTENDED, IntakeRollerState.INTAKEING),
    EXTENDED_NEUTRAL(IntakePivotState.EXTENDED, IntakeRollerState.NEUTRAL),
    
    RETRACTED_NEUTRAL(IntakePivotState.RETRACTED, IntakeRollerState.NEUTRAL),
    RETRACTED_EXTAKEING(IntakePivotState.RETRACTED, IntakeRollerState.EXTAKEING),
    RETRACTED_INTAKEING(IntakePivotState.RETRACTED, IntakeRollerState.INTAKEING),

    AMP_EXTAKEING(IntakePivotState.AMP, IntakeRollerState.EXTAKEING),
    AMP_INTAKEING(IntakePivotState.AMP, IntakeRollerState.INTAKEING),
    AMP_NEUTRAL(IntakePivotState.AMP, IntakeRollerState.NEUTRAL),
    
    NEUTRAL(IntakePivotState.NEUTRAL, IntakeRollerState.NEUTRAL);
    
    private IntakePivotState pivotState;
    private IntakeRollerState rollerState;
    private IntakeState(IntakePivotState piviotState, IntakeRollerState rollerState) {
      this.pivotState = piviotState;
      this.rollerState = rollerState;
    }

    public IntakePivotState getPivotState() {
        return pivotState;
    }

    public IntakeRollerState getRollerState() {
        return rollerState;
    }

    public boolean isInAmpState() {
      return getPivotState() == IntakePivotState.AMP;
    }
  }

  public static enum IntakePivotState {
    EXTENDED(-0.05),//0.1
    AMP(-0.05),
    RETRACTED(0.15),//-0.15
    NEUTRAL(0.0);
    private double motorDutyCycle;
    private IntakePivotState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
        return motorDutyCycle;
    }
  }

  public static enum IntakeRollerState {
    INTAKEING(-0.8),
    EXTAKEING(1.0),
    NEUTRAL(0.0);
    private double motorDutyCycle;
    private IntakeRollerState(double motorDutyCycle) {
      this.motorDutyCycle = motorDutyCycle;
    }

    public double getMotorDutyCycle() {
        return motorDutyCycle;
    }
  }

  public boolean isExtendLimitTriggered() {
    return intakePosSupplier.get() <= PIVIOT_EXTENDED_THRESHOLD;
  }

  public boolean isRetractLimitTriggered() {
    return intakePosSupplier.get() >= PIVIOT_RETRACTED_THRESHOLD;
  }

  public boolean isAmpLimitTriggered() {
    return intakePosSupplier.get() <= PIVOT_AGAINST_AMP_ANGLE_THRESHOLD;
  }

  private void log() {
    BreakerLog.log("Intake/HasNote", hasNote());
    BreakerLog.log("Intake/State", getState().toString());
    BreakerLog.log("Intake/AtPivotSetpoint", isAtTargetState());
    BreakerLog.log("Intake/Threasholds/Extended", isExtendLimitTriggered());
    BreakerLog.log("Intake/Threasholds/Retracted", isExtendLimitTriggered());
    BreakerLog.log("Intake/Threasholds/Amp", isExtendLimitTriggered());
    BreakerLog.log("Intake/Motors/RollerMotor", rollerMotor);
    BreakerLog.log("Intake/Motors/RollerMotor/Temprature", rollerMotor.getDeviceTemp().getValueAsDouble());
    BreakerLog.log("Intake/Motors/LeftPivotMotor", pivotLeft);
    BreakerLog.log("Intake/Motors/rightPivotMotor", pivotRight);
    BreakerLog.log("Intake/PivotEncoder", piviotEncoder);
  }

  @Override
  public void periodic() {
    if (RobotState.isDisabled()) {
      if (isExtendLimitTriggered()) {
        setState(IntakeState.EXTENDED_NEUTRAL);
      } else { 
        setState(IntakeState.RETRACTED_NEUTRAL);
      }
    }

    log();

    if (RobotState.isTeleop() && beamBreak.hasChanged() && beamBreak.isBroken()) {
      new BreakerGamepadTimedRumbleCommand(RobotContainer.controller, 1.5, 0.75, 0.75).schedule();;
    } 
    
    if (!isPivotAmpCurrentLimited && targetState.isInAmpState() && isAmpLimitTriggered()) {
      pivotLeft.getConfigurator().apply(ampCurrentPivotConfig);
      pivotRight.getConfigurator().apply(ampCurrentPivotConfig);
      isPivotAmpCurrentLimited = true;
    } else if (isPivotAmpCurrentLimited && !targetState.isInAmpState()) {
      pivotLeft.getConfigurator().apply(normalCurrentPivotConfig);
      pivotRight.getConfigurator().apply(normalCurrentPivotConfig);
      isPivotAmpCurrentLimited = false;
    }

    piviotDutyCycleControlRequest.withOutput(targetState.getPivotState().getMotorDutyCycle());
    
    pivotLeft.setControl(piviotDutyCycleControlRequest);
    pivotRight.setControl(pivotFollowerRequest);
    rollerMotor.set(targetState.getRollerState().getMotorDutyCycle());
    
  }
  /** Dynaicly enables and disables intake rollers depeding on note visablity */
  public Command smartRollerControlCommand(ZED zed, IntakeState endSetpoint, BooleanSupplier endCondition) {
    Timer timer = new Timer();
    return new FunctionalCommand(()->{
      timer.stop();
      timer.reset();
    }, 
      () -> {
        boolean isApplicableToCurrentState = RobotState.isTeleop();
        if (isApplicableToCurrentState) {
          boolean shouldRun = false;//noteVision.hasTarget();
          if (!shouldRun) {
            for (TrackedObject obj : zed.getTrackedObjects()) {
              double dist = obj.position().getRobotToObject(true).getNorm();
              if (dist <= MAX_SMART_ROLLER_ENABLE_NOTE_DISTANCE.in(Units.Meters)) {
                shouldRun = true;
                break;
              }
            }
          }
          if (!shouldRun) {
            timer.start();
          } else {
            timer.stop();
            timer.reset();
          }

          if ((shouldRun || !timer.hasElapsed(SMART_ROLLER_MAX_WAIT_FOR_DET)) && getState().rollerState != IntakeRollerState.INTAKEING) {
            setState(IntakeState.EXTENDED_INTAKEING);
          } else if (getState().rollerState != IntakeRollerState.NEUTRAL) {
            setState(IntakeState.EXTENDED_NEUTRAL);
          }
        } else {
          timer.stop();
          timer.reset();
          setState(IntakeState.EXTENDED_INTAKEING);
        }
      }, (Boolean cancled) -> setState(endSetpoint), endCondition, this);
  }
}