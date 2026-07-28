// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class rockDestroyerInxder extends SubsystemBase {

  private TalonFX leftRockSmusher = new TalonFX(Constants.LEFT_INDEXER_CANID, CANBus.roboRIO());
  private TalonFX rightRockSmusher = new TalonFX(Constants.RIGHT_INDEXER_CANID, CANBus.roboRIO());
  private TalonFX converyIndex = new TalonFX(Constants.CONVEYOR_CANID, CANBus.roboRIO());

  private TalonFXConfiguration leftIndexerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration rightIndexerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration converyIndexConfig = new TalonFXConfiguration();

   private double converyVoltage = 0.0;
   private double leftRockSmusherVoltage = 0.0;
   private double rightRockSmusherVoltage = 0.0;
   private double converyVoltageBack = 0.0;

   private boolean velocityControl = true;
   private double leftIndexVelocity = 0.0;
   private double rightIndexVelocity = 0.0;

  private final VelocityVoltage v_leftIndexVelVoltage = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage v_rightIndexVelVoltage = new VelocityVoltage(0).withSlot(0);

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Indexer");
  private final DoublePublisher leftIndexerSetPointVelocityPub = table.getDoubleTopic("Left Index Velocity Setpoint").publish(),
                                leftIndexerVelocityPub = table.getDoubleTopic("Left Index Velocity").publish(),
                                leftIndexerCurrentPub = table.getDoubleTopic("Left Index Current").publish(),
                                rightIndexerSetPointVelocityPub = table.getDoubleTopic("Right Index Velocity Setpoint").publish(),
                                rightIndexerVelocityPub = table.getDoubleTopic("Right Index Velocity").publish(),
                                rightIndexerCurrentPub = table.getDoubleTopic("Right Index Current").publish(),
                                spindexerVoltagePub = table.getDoubleTopic("Spindexer Voltage").publish(),
                                spindexerCurrentPub = table.getDoubleTopic("Spindexer Current").publish();

  /** Creates a new rockDestroyerInxder. */
  public rockDestroyerInxder() {
    leftIndexerConfig.MotorOutput.withInverted(Constants.INDEX_LEFT_INVERT);
    rightIndexerConfig.MotorOutput.withInverted(Constants.INDEX_RIGHT_INVERT);
    converyIndexConfig.MotorOutput.withInverted(Constants.INDEX_CONVERY_INVERT);

    leftIndexerConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
    leftIndexerConfig.CurrentLimits.withSupplyCurrentLimit(Constants.INDEXER_LEFT_CURRENT_LIMIT);
    rightIndexerConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
    rightIndexerConfig.CurrentLimits.withSupplyCurrentLimit(Constants.INDEXER_RIGHT_CURRENT_LIMIT);
    converyIndexConfig.CurrentLimits.withSupplyCurrentLimitEnable(true);
    converyIndexConfig.CurrentLimits.withSupplyCurrentLimit(Constants.INDEXER_CONVEYOR_CURRENT_LIMIT);

    leftIndexerConfig.Slot0.kV = 0.1;
    leftIndexerConfig.Slot0.kP = 0.375;
    leftIndexerConfig.Slot0.kI = 0.0;
    leftIndexerConfig.Slot0.kD = 0.0;

    rightIndexerConfig.Slot0.kV = 0.1;
    rightIndexerConfig.Slot0.kP = 0.375;
    rightIndexerConfig.Slot0.kI = 0.0;
    rightIndexerConfig.Slot0.kD = 0.0;

    leftRockSmusher.getConfigurator().apply(leftIndexerConfig);
    rightRockSmusher.getConfigurator().apply(rightIndexerConfig);
    converyIndex.getConfigurator().apply(converyIndexConfig);
  }
  public void setConveryVoltage(double voltageJam){
    this.converyVoltage = voltageJam;
  }

   public void setLeftRockSumusherVoltage(double voltagePeanut){
    this.leftRockSmusherVoltage = voltagePeanut;
    velocityControl = false;
  }

   public void setRightRockSmusherVoltage(double voltageButter){
    this.rightRockSmusherVoltage = voltageButter;
    velocityControl = false;
  }

  public void setConveryVoltageBack(double voltageBread){
    this.converyVoltageBack = voltageBread;
  }

  public void setLeftIndexVelocity(double leftVelo){
    this.leftIndexVelocity = leftVelo;
    velocityControl = true;
  }

  public void setRightIndexVelocity(double rightVelo){
    this.rightIndexVelocity = rightVelo;
    velocityControl = true;
  }

  public boolean leftIndexerAtVelocity(){
    if(Math.abs(leftRockSmusher.getVelocity().getValueAsDouble() - leftIndexVelocity) < Constants.INDEXER_LEFT_ALLOWABLE_ERROR){
      return true;
    }
    return false;
  }

  public boolean rightIndexerAtVelocity(){
    if(Math.abs(rightRockSmusher.getVelocity().getValueAsDouble() - rightIndexVelocity) < Constants.INDEXER_RIGHT_ALLOWABLE_ERROR){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    converyIndex.setVoltage(Constants.MAX_CONVEYOR_VOLTAGE * converyVoltage);
    if(!velocityControl){
      leftRockSmusher.setVoltage(Constants.MAX_LEFT_SMUSHER_VOLTAGE * leftRockSmusherVoltage);
      rightRockSmusher.setVoltage(Constants.MAX_RIGHT_SMUSHER_VOLTAGE * rightRockSmusherVoltage);
    }
    else{
      leftRockSmusher.setControl(v_leftIndexVelVoltage.withVelocity(leftIndexVelocity).withEnableFOC(true));
      rightRockSmusher.setControl(v_rightIndexVelVoltage.withVelocity(rightIndexVelocity).withEnableFOC(true));
    }
    
    leftIndexerVelocityPub.set(leftRockSmusher.getVelocity().getValueAsDouble());
    leftIndexerSetPointVelocityPub.set(leftIndexVelocity);
    leftIndexerCurrentPub.set(leftRockSmusher.getSupplyCurrent().getValueAsDouble());
    rightIndexerVelocityPub.set(rightRockSmusher.getVelocity().getValueAsDouble());
    rightIndexerSetPointVelocityPub.set(rightIndexVelocity);
    rightIndexerCurrentPub.set(rightRockSmusher.getSupplyCurrent().getValueAsDouble());
    spindexerCurrentPub.set(converyIndex.getSupplyCurrent().getValueAsDouble());
    spindexerVoltagePub.set(converyIndex.getMotorVoltage().getValueAsDouble());

    // This method will be called once per scheduler run
  }
}
