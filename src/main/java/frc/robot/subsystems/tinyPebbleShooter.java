// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class tinyPebbleShooter extends SubsystemBase {

  private TalonFX lazySusanMotor = new TalonFX(Constants.lazyShoot, "rio");
  private TalonFX leftSlingShot = new TalonFX(Constants.leftShoot, "rio");
  private TalonFX rightSlingShot = new TalonFX(Constants.rightShoot, "rio");

  private double rotateVoltage = 0.0;
  private double leftVoltage = 0.0;
  private double rightVoltage = 0.0;

  /** Creates a new tinyPebbleShooter. */
  public tinyPebbleShooter() {
    
  }

  public void setRotateVoltage(double voltage){
    this.rotateVoltage = voltage;
  }

  public void setLeftSlingShotVoltage(double leftvoltage){
    this.leftVoltage = leftvoltage;
  }

  public void setRightSlingShotVoltage(double rightvoltage){
    this.rightVoltage = rightvoltage;
  }

  @Override
  public void periodic() {
    lazySusanMotor.setVoltage(Constants.maxVoltage * rotateVoltage);
    lazySusanMotor.setVoltage(Constants.leftSlingMaxVoltage * leftVoltage);
    lazySusanMotor.setVoltage(Constants.rightSlingMaxVoltage * rightVoltage);
    // This method will be called once per scheduler run
  }
}
