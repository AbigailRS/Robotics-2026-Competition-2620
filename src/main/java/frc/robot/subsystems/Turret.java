// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private TalonFX rotateTurret;
  private TalonFXConfiguration rotateTurretConfig;
  private double rotateTurretVoltage;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Turret");
  private final DoublePublisher pri_ll_tx_log = table.getDoubleTopic("Primary LL tx").publish(),
                                sec_ll_tx_log = table.getDoubleTopic("Secondary LL tx").publish(),
                                turret_voltage_log = table.getDoubleTopic("Turret Voltage").publish(),
                                turret_current_log = table.getDoubleTopic("Turret Current").publish();

  public Turret() {
    rotateTurret = new TalonFX(Constants.TURRET_CANID, CANBus.roboRIO());
    rotateTurretConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(Constants.TURRET_RAMPRATE);
    rotateTurretConfig.MotorOutput.withInverted(Constants.TURRET_INVERSION);
    rotateTurretConfig.CurrentLimits.withStatorCurrentLimitEnable(true);
    rotateTurretConfig.CurrentLimits.withStatorCurrentLimit(Constants.TURRET_CURRENT_LIMIT);
    rotateTurret.setNeutralMode(Constants.TURRET_NEUTRALMODE);
    rotateTurret.getConfigurator().apply(rotateTurretConfig);
  }

  public void setTurretVoltage(double voltage){
    rotateTurretVoltage = voltage;
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
    if(rotateTurret.getStatorCurrent().getValueAsDouble() > Constants.TURRET_STALL_CURRENT && Math.abs(rotateTurret.getVelocity().getValueAsDouble()) > Constants.TURRET_STALL_VELOCITY){
      return true;
    }
    else{
      return false;
    }
  }

  @Override
  public void periodic() {
    rotateTurret.setVoltage(rotateTurretVoltage);
    updateLogging();
    
  }

  public void updateLogging(){

  }

}
