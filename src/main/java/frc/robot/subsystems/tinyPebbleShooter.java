// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class tinyPebbleShooter extends SubsystemBase {

  private TalonFX lazySusanMotor = new TalonFX(Constants.LAZY_SHOOT_CANID, CANBus.roboRIO());
  private TalonFX leftSlingShot = new TalonFX(Constants.LEFT_SHOOT_CANID, CANBus.roboRIO());
  private TalonFX rightSlingShot = new TalonFX(Constants.RIGHT_SHOOT_CANID, CANBus.roboRIO());

  private double rotateVoltage = 0.0;
  private double leftVoltage = 0.0;
  private double rightVoltage = 0.0;
  private double leftVelocity = 0.0;
  private double rightVelocity = 0.0;

  Slot1Configs slot1Configs = new Slot1Configs();
  Slot2Configs slot2Configs = new Slot2Configs();
  final VelocityVoltage v_rightVelocityVoltage = new VelocityVoltage(0).withSlot(1);
  final VelocityVoltage v_leftVelocityVoltage = new VelocityVoltage(0).withSlot(2);

  ClosedLoopRampsConfigs ramprateConfig = new ClosedLoopRampsConfigs();

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Shooter");
  private final DoublePublisher shooterLeftVelocitySetPointPublisher = table.getDoubleTopic("Shooter Left Velocity SetPoint").publish(), shooterRightVelocitySetPointPublisher = table.getDoubleTopic("Shooter Right Velocity SetPoint").publish(),
                                shooterLeftVelocityPublisher = table.getDoubleTopic("Shooter Left Velocity").publish(), shooterRightVelocityPublisher = table.getDoubleTopic("Shooter Right Velocity").publish();

  /** Creates a new tinyPebbleShooter. */
  public tinyPebbleShooter() {
    slot1Configs.kP = 2.4;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0.5;
    rightSlingShot.getConfigurator().apply(slot1Configs);

    slot2Configs.kP = 2.4;
    slot2Configs.kI = 0;
    slot2Configs.kD = 0.5;
    leftSlingShot.getConfigurator().apply(slot2Configs);
    ramprateConfig.withVoltageClosedLoopRampPeriod(50);
    leftSlingShot.getConfigurator().apply(ramprateConfig);
    rightSlingShot.getConfigurator().apply(ramprateConfig);

  
    
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

  public void setLeftSlingVelocity(double velocity){
    this.leftVelocity = velocity;
  }

  public void setRightSlingVelocity(double velocity){
    this.rightVelocity = velocity;
  }

  @Override
  public void periodic() {
    lazySusanMotor.setVoltage(Constants.MAX_VOLTAGE * rotateVoltage);
    shooterLeftVelocitySetPointPublisher.set(leftVelocity);
    shooterLeftVelocityPublisher.set(leftSlingShot.getVelocity().getValueAsDouble());
    shooterRightVelocitySetPointPublisher.set(rightVelocity);
    shooterRightVelocityPublisher.set(rightSlingShot.getVelocity().getValueAsDouble());
    leftSlingShot.setControl(v_leftVelocityVoltage.withVelocity(leftVelocity));
    rightSlingShot.setControl(v_rightVelocityVoltage.withVelocity(rightVelocity));
    //leftSlingShot.setControl(v_rightVelocityVoltage.withVelocity(rightVelocity));
    //leftSlingShot.setVoltage(Constants.LEFT_SLING_MAX_VOLTAGE * leftVoltage);
    //rightSlingShot.setVoltage(Constants.RIGHT_SLING_MAX_VOLTAGE * rightVoltage);
    // This method will be called once per scheduler run
  }
}
