// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.Shooter.leftSlingVelocity;

public class tinyPebbleShooter extends SubsystemBase {

  

  private TalonFX leftSlingShot = new TalonFX(Constants.LEFT_SHOOT_CANID, CANBus.roboRIO());
  private TalonFX rightSlingShot = new TalonFX(Constants.RIGHT_SHOOT_CANID, CANBus.roboRIO());

  private TalonFXConfiguration leftSlingShotConfig = new TalonFXConfiguration();
  private TalonFXConfiguration rightSlingShotConfig = new TalonFXConfiguration();

  private LaserCan leftLaserCan = new LaserCan(Constants.LEFT_LASERCAN_CANID);
  private LaserCan rightLaserCan = new LaserCan(Constants.RIGHT_LASERCAN_CANID);

  //private double rotateVoltage = 0.0;
  private double leftVoltage = 0.0;
  private double rightVoltage = 0.0;
  private double leftVelocity = 0.0;
  private double rightVelocity = 0.0;

  private int shotCount = 0, laserCanLeftLastMeasurement = 0, laserCanRightLastMeasurement = 0;

  private boolean leftShooterVeloControlMode = false, rightShooterVeloControlMode = false;


  Slot1Configs slot1Configs = new Slot1Configs();
  Slot1Configs rightSlotConfigs = new Slot1Configs();
  final VelocityVoltage v_rightVelocityVoltage = new VelocityVoltage(0).withSlot(1);
  final VelocityVoltage v_leftVelocityVoltage = new VelocityVoltage(0).withSlot(1);

  // ClosedLoopRampsConfigs ramprateConfig = new ClosedLoopRampsConfigs();

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Shooter");
  private final DoublePublisher shooterLeftVelocitySetPointPublisher = table.getDoubleTopic("Shooter Left Velocity SetPoint").publish(), shooterRightVelocitySetPointPublisher = table.getDoubleTopic("Shooter Right Velocity SetPoint").publish(),
                                shooterLeftVelocityPublisher = table.getDoubleTopic("Shooter Left Velocity").publish(), shooterRightVelocityPublisher = table.getDoubleTopic("Shooter Right Velocity").publish(),
                                shooterLeftVoltagePublisher = table.getDoubleTopic("Shooter Left Voltage").publish(), shooterRightVoltagePublisher = table.getDoubleTopic("Shooter Right Voltage").publish();

  private final BooleanPublisher atLeftVeloPub = table.getBooleanTopic("At Left Velo").publish(), atRightVeloPub = table.getBooleanTopic("At Right Velo").publish();

  /** Creates a new tinyPebbleShooter. */
  public tinyPebbleShooter() {
    slot1Configs.kV = 0.10;
    slot1Configs.kP = 0.4;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0.0;
    rightSlotConfigs.kV = 0.10;
    rightSlotConfigs.kP = 0.4;
    rightSlotConfigs.kI = 0.0;
    rightSlotConfigs.kD = 0.0;
    rightSlingShotConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    leftSlingShotConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    rightSlingShotConfig.withSlot1(rightSlotConfigs);
    leftSlingShotConfig.withSlot1(slot1Configs);
    rightSlingShot.getConfigurator().apply(rightSlingShotConfig);
    leftSlingShot.getConfigurator().apply(leftSlingShotConfig);

    //ramprateConfig.withVoltageClosedLoopRampPeriod(50);
    // leftSlingShot.getConfigurator().apply(ramprateConfig);
    // rightSlingShot.getConfigurator().apply(ramprateConfig);

    //   leftLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
    //   leftLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    //   leftLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    //   rightLaserCan.setRangingMode(LaserCan.RangingMode.SHORT);
    //   rightLaserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    //   rightLaserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    // }catch (ConfigurationFailedException e) {
    //   System.out.println("Configuration failed! " + e);
    // }

  }


  public void setLeftSlingShotVoltage(double leftvoltage){
    this.leftVoltage = leftvoltage;
    leftShooterVeloControlMode = false;
  }

  public void setRightSlingShotVoltage(double rightvoltage){
    this.rightVoltage = rightvoltage;
    rightShooterVeloControlMode = false;
  }

  public void setLeftSlingVelocity(double velocity){
    this.leftVelocity = velocity;
    leftShooterVeloControlMode = true;
  }

  public void setRightSlingVelocity(double velocity){
    this.rightVelocity = velocity;
    rightShooterVeloControlMode = true;
  }

  public int getShotCount(){
    return shotCount;
  }

  public void setShotCount(int value){
    shotCount = value;
  }

  public boolean atLeftShootVelocity(){
    if(Math.abs(leftSlingShot.getVelocity().getValueAsDouble() - leftVelocity) < Constants.SHOOTER_LEFT_ALLOWABLE_ERROR){
      return true;
    }
    return false;
  }

  public boolean atRightShootVelocity(){
    if(Math.abs(rightSlingShot.getVelocity().getValueAsDouble() - rightVelocity) < Constants.SHOOTER_RIGHT_ALLOWABLE_ERROR){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {

    if(leftShooterVeloControlMode){
      leftSlingShot.setControl(v_leftVelocityVoltage.withVelocity(leftVelocity).withEnableFOC(true));
    }
    else{
      leftSlingShot.setVoltage(Constants.LEFT_SLING_MAX_VOLTAGE * leftVoltage);
    }
    if(rightShooterVeloControlMode){
      rightSlingShot.setControl(v_rightVelocityVoltage.withVelocity(rightVelocity).withEnableFOC(true));
    }
    else{
      rightSlingShot.setVoltage(Constants.RIGHT_SLING_MAX_VOLTAGE * rightVoltage);
    }
    // This method will be called once per scheduler run
    this.updateLogging();

  }

  public void updateLogging(){
    shooterLeftVelocitySetPointPublisher.set(leftVelocity);
    shooterLeftVelocityPublisher.set(leftSlingShot.getVelocity().getValueAsDouble());
    shooterRightVelocitySetPointPublisher.set(rightVelocity);
    shooterRightVelocityPublisher.set(rightSlingShot.getVelocity().getValueAsDouble());
    shooterLeftVoltagePublisher.set(leftSlingShot.getMotorVoltage().getValueAsDouble());
    shooterRightVoltagePublisher.set(rightSlingShot.getMotorVoltage().getValueAsDouble());
    atLeftVeloPub.set(this.atLeftShootVelocity());
    atRightVeloPub.set(this.atRightShootVelocity());
  }
}
