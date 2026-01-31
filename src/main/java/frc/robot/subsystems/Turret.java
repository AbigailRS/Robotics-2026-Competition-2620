// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  public TalonFX rotateTurret;
  public TalonFXConfiguration rotateTurretConfig;
  public PIDController rotateTurretPIDController;
  public double rotateTurretVoltage;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Turret");
  private final DoublePublisher priTurretLLTx = table.getDoubleTopic("turretLLTx").publish(),
                                priTurretLLTxPID = table.getDoubleTopic("turretRotatePIDOutput").publish(),
                                secTurretLLTx = table.getDoubleTopic("turretRotatePIDOutput").publish(),
                                secTurretLLTxPID = table.getDoubleTopic("turretRotatePIDOutput").publish();

  public Turret() {
    rotateTurret = new TalonFX(Constants.TURRET_ROTATE_ID, CANBus.roboRIO());
    rotateTurretPIDController = new PIDController(1.0, 0.0, 0.0);
    
  }

  public void setTurretRotateOnTarget(){
    if(LimelightHelpers.getTargetCount(Constants.PRIMARY_LIMELIGHT) > 0){
      rotateTurretVoltage = rotateTurretPIDController.calculate(LimelightHelpers.getTX(Constants.PRIMARY_LIMELIGHT), 0.0);
    }
    else if(LimelightHelpers.getTargetCount(Constants.SECONDARY_LIMELIGHT) > 0){
      rotateTurretVoltage = rotateTurretPIDController.calculate(LimelightHelpers.getTX(Constants.SECONDARY_LIMELIGHT), 0.0);
    }
    else{
      rotateTurretVoltage = 0.0;
    }
    
  }

  @Override
  public void periodic() {
    rotateTurret.setVoltage(rotateTurretVoltage);
    priTurretLLTx.set(LimelightHelpers.getTX(Constants.PRIMARY_LIMELIGHT));
    priTurretLLTxPID.set(rotateTurretPIDController.calculate(LimelightHelpers.getTX(Constants.PRIMARY_LIMELIGHT), 0.0));
    secTurretLLTx.set(LimelightHelpers.getTX(Constants.SECONDARY_LIMELIGHT));
    secTurretLLTxPID.set(rotateTurretPIDController.calculate(LimelightHelpers.getTX(Constants.SECONDARY_LIMELIGHT), 0.0));
    
  }

}
