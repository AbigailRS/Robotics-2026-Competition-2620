// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class bigRockIntake extends SubsystemBase {

  private TalonFX rockGrabber = new TalonFX(Constants.LOWER_WHEEL_INTAKE_CANID, CANBus.roboRIO());
  private TalonFX rockPusher = new TalonFX(Constants.UPPER_WHEEL_INTAKE_CANID, CANBus.roboRIO());

  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private TalonFXConfiguration extendConfig = new TalonFXConfiguration();

  private double extendVoltage = 0.0;
  private double intakeVoltage = 0.0;

  private double extendPos = 0.0;

  private boolean posControlOn = false;

  final PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(1);

  Slot1Configs configs = new Slot1Configs();

   private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Indexer");
  private final DoublePublisher IntakeVoltageSetPublisher = table.getDoubleTopic("Intake Voltage SetPoint").publish(),
                                ExtendVoltageSetPublisher = table.getDoubleTopic("Extend Voltage SetPoint").publish(),
                                IntakeVoltagePublisher = table.getDoubleTopic("Intake Voltage").publish(),
                                ExtendVoltagePublisher = table.getDoubleTopic("Extend Voltage").publish(),
                                intakeExtendPostionForwardsPublisher = table.getDoubleTopic("Extend Forwards").publish(),
                                intakeExtendPostionBackwardsPublisher = table.getDoubleTopic("Extend Backwards").publish();
  /** Creates a new rockDestroyerInxder. */

  /** Creates a new bigRockIntake. */
  public bigRockIntake() {

    configs.kV = 0.11;
    configs.kP = 0.025;
    configs.kI = 0;
    configs.kD = 0.0;
    extendConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    extendConfig.withSlot1(configs);
    rockGrabber.getConfigurator().apply(intakeConfig);
    rockPusher.getConfigurator().apply(extendConfig);
  }

  public void setExtendVoltage(double voltageEX){
      this.extendVoltage = voltageEX;
    }

  public void setExtendPosition(double extendPos){
    this.extendPos = extendPos;
  }

  public void setIntakeVoltage(double voltageIN){
      this.intakeVoltage = voltageIN;
  }

  public boolean intakeInPosition(){
    if(Math.abs(rockPusher.getPosition().getValueAsDouble() - extendPos) < Constants.INTAKE_IN_POSITION_ERROR){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    rockGrabber.setVoltage(Constants.MAX_INTAKE_VOLTAGE * intakeVoltage   /*Constants.INTAKE_VOLTAGE_PERCENTAGE*/);

    IntakeVoltageSetPublisher.set(intakeVoltage);
    IntakeVoltagePublisher.set(rockGrabber.getSupplyVoltage().getValueAsDouble());
    ExtendVoltageSetPublisher.set(extendVoltage);
    ExtendVoltagePublisher.set(rockPusher.getSupplyVoltage().getValueAsDouble());
    intakeExtendPostionBackwardsPublisher.set(rockGrabber.getPosition().getValueAsDouble());
    intakeExtendPostionForwardsPublisher.set(rockGrabber.getPosition().getValueAsDouble());

    if(posControlOn){
      rockPusher.setControl(positionVoltage.withPosition(extendPos).withEnableFOC(true));
    }
    else{
      rockPusher.setVoltage(Constants.MAX_EXTEND_VOLTAGE * extendVoltage);
    }


    // This method will be called once per scheduler run
  }
}
