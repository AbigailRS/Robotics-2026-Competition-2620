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

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Indexer");
  private final DoublePublisher indexLeftTowerVoltageSetPublisher = table.getDoubleTopic("Index Left Voltage SetPoint").publish(),
                                indexRightTowerVoltageSetPublisher = table.getDoubleTopic("Index Right Voltage SetPoint").publish(),
                                indexLeftTowerVoltagePublisher = table.getDoubleTopic("Index Left Voltage").publish(),
                                indexRightTowerVoltagePublisher = table.getDoubleTopic("Index Right Voltage").publish();

  /** Creates a new rockDestroyerInxder. */
  public rockDestroyerInxder() {
    leftIndexerConfig.MotorOutput.withInverted(Constants.INDEX_LEFT_INVERT);
    rightIndexerConfig.MotorOutput.withInverted(Constants.INDEX_RIGHT_INVERT);
    converyIndexConfig.MotorOutput.withInverted(Constants.INDEX_CONVERY_INVERT);

    leftRockSmusher.getConfigurator().apply(leftIndexerConfig);
    rightRockSmusher.getConfigurator().apply(rightIndexerConfig);
    converyIndex.getConfigurator().apply(converyIndexConfig);
  }
  public void setConveryVoltage(double voltageJam){
    this.converyVoltage = voltageJam;
  }

   public void setLeftRockSumusherVoltage(double voltagePeanut){
    this.leftRockSmusherVoltage = voltagePeanut;
  }

   public void setRightRockSmusherVoltage(double voltageButter){
    this.rightRockSmusherVoltage = voltageButter;
  }

  public void setConveryVoltageBack(double voltageBread){
    this.converyVoltageBack = voltageBread;
  }


  @Override
  public void periodic() {
    converyIndex.setVoltage(Constants.MAX_CONVEYOR_VOLTAGE * converyVoltage);
    leftRockSmusher.setVoltage(Constants.MAX_LEFT_SMUSHER_VOLTAGE * leftRockSmusherVoltage);
    rightRockSmusher.setVoltage(Constants.MAX_RIGHT_SMUSHER_VOLTAGE * rightRockSmusherVoltage);
    
    indexLeftTowerVoltageSetPublisher.set(leftRockSmusherVoltage);
    indexLeftTowerVoltagePublisher.set(leftRockSmusher.getVelocity().getValueAsDouble());
    indexRightTowerVoltageSetPublisher.set(rightRockSmusherVoltage);
    indexRightTowerVoltagePublisher.set(rightRockSmusher.getVelocity().getValueAsDouble());

    // This method will be called once per scheduler run
  }
}
