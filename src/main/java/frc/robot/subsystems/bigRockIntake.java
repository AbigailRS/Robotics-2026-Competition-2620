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

public class bigRockIntake extends SubsystemBase {

  private TalonFX rockGrabber = new TalonFX(Constants.LOWER_WHEEL_INTAKE_CANID, CANBus.roboRIO());
  private TalonFX rockPusher = new TalonFX(Constants.UPPER_WHEEL_INTAKE_CANID, CANBus.roboRIO());

  private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private TalonFXConfiguration extendConfig = new TalonFXConfiguration();

  private double extendVoltage = 0.0;
  private double intakeVoltage = 0.0;

   private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Indexer");
  private final DoublePublisher IntakeVoltageSetPublisher = table.getDoubleTopic("Intake Voltage SetPoint").publish(),
                                ExtendVoltageSetPublisher = table.getDoubleTopic("Extend Voltage SetPoint").publish(),
                                IntakeVoltagePublisher = table.getDoubleTopic("Intake Voltage").publish(),
                                ExtendVoltagePublisher = table.getDoubleTopic("Extend Voltage").publish();

  /** Creates a new rockDestroyerInxder. */

  /** Creates a new bigRockIntake. */
  public bigRockIntake() {

    rockGrabber.getConfigurator().apply(intakeConfig);
    rockPusher.getConfigurator().apply(extendConfig);
  }

  public void setExtendVoltage(double voltageEX){
      this.extendVoltage = voltageEX;
    }

      public void setIntakeVoltage(double voltageIN){
      this.intakeVoltage = voltageIN;
    }

  @Override
  public void periodic() {
    rockGrabber.setVoltage(Constants.MAX_INTAKE_VOLTAGE * intakeVoltage   /*Constants.INTAKE_VOLTAGE_PERCENTAGE*/);
    rockPusher.setVoltage(Constants.MAX_EXTEND_VOLTAGE * extendVoltage /*Constants.EXTEND_VOLTAGE_PERCENTAGE*/);

    IntakeVoltageSetPublisher.set(intakeVoltage);
    IntakeVoltagePublisher.set(rockGrabber.getSupplyVoltage().getValueAsDouble());
    ExtendVoltageSetPublisher.set(extendVoltage);
    ExtendVoltagePublisher.set(rockPusher.getSupplyVoltage().getValueAsDouble());


    // This method will be called once per scheduler run
  }
}
