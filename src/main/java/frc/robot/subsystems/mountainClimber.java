// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class mountainClimber extends SubsystemBase {

  private TalonFX mountainClimbingLeft = new TalonFX(Constants.climbLeft, "rio");
  private TalonFX mountainClimbingRight = new TalonFX(Constants.climbRight, "rio");

  private double climbingLeftVoltage = 0.0;
  private double climbingRightVoltage = 0.0;

  /** Creates a new mountainClimber. */
  public mountainClimber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
