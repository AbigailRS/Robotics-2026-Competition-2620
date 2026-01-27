// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class bigRockIntake extends SubsystemBase {

  private TalonFX rockGrabber = new TalonFX(Constants.lowerWheelIntake, "rio");
  private TalonFX rockPusher = new TalonFX(Constants.upperWheelIntake, "rio");

  private double extendVoltage = 0.0;
  private double intakeVoltage = 0.0;

  /** Creates a new bigRockIntake. */
  public bigRockIntake() {
  }

  public void setExtendVoltage(double voltageEX){
      this.extendVoltage = voltageEX;
    }

      public void setIntakeVoltage(double voltageIN){
      this.intakeVoltage = voltageIN;
    }

  @Override
  public void periodic() {
    rockGrabber.setVoltage(Constants.maxIntakeVoltage * Constants.intakeVoltagePercentage);
    rockPusher.setVoltage(Constants.maxExtendVoltage * Constants.extendVoltagePercentage);


    // This method will be called once per scheduler run
  }
}
