// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenTest extends SubsystemBase {
  /** Creates a new KrakenTest. */

  private TalonFX kraken = new TalonFX(20, "rio");
  private double krakenSpeed = 0.0, krakenPosition = 0.0;
  private boolean positionControl = false;

  Slot0Configs slot0Configs = new Slot0Configs();
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

  public KrakenTest() {
    slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
    kraken.getConfigurator().apply(slot0Configs);
  }

  public void setKrakenSpeed(double speed){
    krakenSpeed = speed;
    positionControl = false;
  }

  public void setKrakenPosition(double position){
    this.krakenPosition = position;
    positionControl = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(positionControl){
        kraken.setControl(m_request.withPosition(krakenPosition));
    }
    else{
        kraken.setVoltage(krakenSpeed);
    }
  }
}
