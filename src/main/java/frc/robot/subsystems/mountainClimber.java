// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class mountainClimber extends SubsystemBase {

  private TalonFX mountainClimbingLeft = new TalonFX(Constants.CLIMB_LEFT_CANID, CANBus.roboRIO());
  private TalonFX mountainClimbingRight = new TalonFX(Constants.CLIMB_RIGHT_CANID, CANBus.roboRIO());

  private TalonFXConfiguration leftClimbConfig = new TalonFXConfiguration();
  private TalonFXConfiguration rightClimbConfig = new TalonFXConfiguration();

  /* 
  private double climbingLeftVoltage = 0.0;
  private double climbingRightVoltage = 0.0;
  */

  private double climbingLeftPosition = 0.0;
  private double climbingRightPosition = 0.0;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Indexer");
  private final DoublePublisher climbLeftMotorSetPositionPublisher = table.getDoubleTopic("Climb Left Position SetPoint").publish(),
                                climbRightMotorSetPositionPublisher = table.getDoubleTopic("Climb Right Position SetPoint").publish(),
                                climbLeftMotorPositionPublisher = table.getDoubleTopic("Climb Left Position").publish(),
                                climbRightMotorPositionPublisher = table.getDoubleTopic("Climb Right Position").publish();

  /** Creates a new mountainClimber. */
  public mountainClimber() {


    mountainClimbingLeft.getConfigurator().apply(leftClimbConfig);
    mountainClimbingRight.getConfigurator().apply(rightClimbConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // climbLeftMotorSetPositionPublisher.set(climbingLeftPosition);
    // climbLeftMotorPositionPublisher.set(mountainClimbingLeft.getPosition().getValueAsDouble());
    // climbRightMotorSetPositionPublisher.set(climbingRightPosition);
    // climbRightMotorPositionPublisher.set(mountainClimbingRight.getPosition().getValueAsDouble());
  }
}
