// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private TalonFX rotateTurret;
  private TalonFXConfiguration rotateTurretConfig;
  private double rotateTurretVoltage;
  private CANcoder cancoder;
  private CANcoderConfiguration cancoderConfig;
  private boolean disableTurret;
  private Timer lastSightedTimer = new Timer();
  private boolean lastSeenDirectionLeft = true;
  private boolean manualRotation = false;
  private boolean positionControlMode = true;
  private double turretPosition = 0.0;

  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Turret");
  private final DoublePublisher pri_ll_ty_pub = table.getDoubleTopic("Primary LL tx").publish(),
                                sec_ll_ty_pub = table.getDoubleTopic("Secondary LL tx").publish(),
                                turret_voltage_pub = table.getDoubleTopic("Turret Voltage").publish(),
                                turret_current_pub = table.getDoubleTopic("Turret Current").publish(),
                                turret_velocity_pub = table.getDoubleTopic("Turret Velocity").publish(),
                                encoder_setpoint_pub = table.getDoubleTopic("Turret Setpoint Pos").publish(),
                                encoder_pos_pub = table.getDoubleTopic("Turret Encoder Pos").publish();
  private final BooleanPublisher stalled_pub = table.getBooleanTopic("isStalled").publish();

  public Turret() {
    rotateTurret = new TalonFX(Constants.TURRET_CANID, CANBus.roboRIO());
    rotateTurretConfig = new TalonFXConfiguration();
    rotateTurretConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(Constants.TURRET_RAMPRATE);
    rotateTurretConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    rotateTurretConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
    rotateTurretConfig.CurrentLimits.withStatorCurrentLimit(Constants.TURRET_CURRENT_LIMIT);
    rotateTurret.setNeutralMode(Constants.TURRET_NEUTRALMODE);

    rotateTurretConfig.Slot0.kV = 0.5;
    rotateTurretConfig.Slot0.kP = 3.0;
    rotateTurretConfig.Slot0.kI = 0.0;
    rotateTurretConfig.Slot0.kD = 0.0;
    
    rotateTurretConfig.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    rotateTurretConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(170.0);
    rotateTurretConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
    rotateTurretConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-170.0);

    rotateTurret.getConfigurator().apply(rotateTurretConfig);

  }

  public void setTurretVoltage(double voltage){
    rotateTurretVoltage = voltage;
    positionControlMode = false;
  }

  public void setTurretPosition(double position){
    turretPosition = position * Constants.TURRET_MOTOR_TO_TURRET_RATIO;
    turretPosition = turretPosition / 360;
    positionControlMode = true;
    
  }

  public boolean priLLHasTarget(){
    if(LimelightHelpers.getTV(Constants.PRIMARY_LL_NAME) && LimelightHelpers.getLatency_Capture(Constants.PRIMARY_LL_NAME) > 0){
      return true;
    }
    return false;
  }

  public boolean secLLHasTarget(){
    if(LimelightHelpers.getTV(Constants.SECONDARY_LL_NAME) && LimelightHelpers.getLatency_Capture(Constants.SECONDARY_LL_NAME) > 0){
      return true;
    }
    return false;
  }

  public void updateTargetTags(){
    if(!DriverStation.getAlliance().isEmpty()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.PRIMARY_LL_NAME, new int[]{2, 5, 10});
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.SECONDARY_LL_NAME, new int[]{2, 5, 10});
      }
      else{
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.PRIMARY_LL_NAME, new int[]{18, 21, 26});
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.SECONDARY_LL_NAME, new int[]{18, 21, 26});
      }
    }
  }

  public boolean isStalled(){
    if(rotateTurret.getStatorCurrent().getValueAsDouble() > Constants.TURRET_STALL_CURRENT && Math.abs(rotateTurret.getVelocity().getValueAsDouble()) < Constants.TURRET_STALL_VELOCITY){
      return true;
    }
    else{
      return false;
    }
  }

  public void manualRun(double voltage){
    this.rotateTurretVoltage = voltage;
  }

  public void setDisableTurret(boolean disableTurret){
    this.disableTurret = disableTurret;
  }
  
  public boolean getIfTurretDisabled(){
    return this.disableTurret;
  }

  public double getTimeSinceLastSighted(){
    return lastSightedTimer.get();
  }

  public boolean getLastSeenDriectionLeft(){
    return lastSeenDirectionLeft;
  }

  public void setManualRotate(boolean manualRotateValue){
    manualRotation = manualRotateValue;
  }

  public boolean manualRotateEnabled(){
    return manualRotation;
  }

  public void setRotateEncoder(double encoderValue){
    rotateTurret.setPosition(encoderValue);
  }

  @Override
  public void periodic() {
    if(positionControlMode){
      rotateTurret.setControl(m_positionVoltage.withPosition(turretPosition).withEnableFOC(true));
    }
    else{
      rotateTurret.setVoltage(rotateTurretVoltage);
    }

    if(!priLLHasTarget() && !secLLHasTarget()){
      lastSightedTimer.start();
    }
    else{
      lastSightedTimer.reset();
      lastSightedTimer.stop();
    }
    //updateLogging();
    
  }

  public void updateLogging(){
    pri_ll_ty_pub.set(LimelightHelpers.getTY(Constants.PRIMARY_LL_NAME));
    sec_ll_ty_pub.set(LimelightHelpers.getTY(Constants.SECONDARY_LL_NAME));
    turret_voltage_pub.set(rotateTurret.getMotorVoltage().getValueAsDouble());
    turret_current_pub.set(rotateTurret.getStatorCurrent().getValueAsDouble());
    turret_velocity_pub.set(rotateTurret.getVelocity().getValueAsDouble());
    encoder_setpoint_pub.set(turretPosition);
    encoder_pos_pub.set(rotateTurret.getPosition().getValueAsDouble());
    stalled_pub.set(isStalled());
  }

}
