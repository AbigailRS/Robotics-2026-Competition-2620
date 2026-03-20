// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hoods extends SubsystemBase {
  /** Creates a new Hoods. */

  private Servo leftHoodieServo = new Servo(Constants.LEFT_HOOD_SERVO);
  private Servo rightHoodieServo = new Servo(Constants.RIGHT_HOOD_SERVO);

  private InterpolatingDoubleTreeMap leftServoAngleMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap rightServoAngleMap = new InterpolatingDoubleTreeMap();


  private double leftServoPos, rightServoPos;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Hoods");
  private final DoublePublisher hoodLeftSetPositionPublisher = table.getDoubleTopic("Hood Left Position SetPoint").publish(),
                                hoodRightSetPositionPublisher = table.getDoubleTopic("Hood Right Position SetPoint").publish(),
                                hoodLeftPositionPublisher = table.getDoubleTopic("Hood Left Position").publish(),
                                hoodRightPositionPublisher = table.getDoubleTopic("Hood Right Position").publish();

  public Hoods() {
    leftServoPos = Constants.HOOD_LEFT_LOW_POSITION;
    rightServoPos = Constants.HOOD_RIGHT_LOW_POSITION;

    leftServoAngleMap.put(0.0, 0.0);
    leftServoAngleMap.put(5.0, 0.1);
    leftServoAngleMap.put(10.0, 0.22);
    leftServoAngleMap.put(15.0, 0.375);
    leftServoAngleMap.put(20.0, 0.5);
    leftServoAngleMap.put(25.0, 0.75);
    leftServoAngleMap.put(30.0, 0.9);
    leftServoAngleMap.put(35.0, 1.0);

    rightServoAngleMap.put(0.0, 0.0);
    rightServoAngleMap.put(5.0, 0.1);
    rightServoAngleMap.put(10.0, 0.22);
    rightServoAngleMap.put(15.0, 0.375);
    rightServoAngleMap.put(20.0, 0.5);
    rightServoAngleMap.put(25.0, 0.75);
    rightServoAngleMap.put(30.0, 0.9);
    rightServoAngleMap.put(35.0, 1.0);

  }

  public void setLeftServoPosition(double position){
    this.leftServoPos = position;
  }

  public void setRightServoPosition(double position){
    this.rightServoPos = position;
  }

  public void setLeftServoAngle(double angleDegrees){
    this.leftServoPos = leftServoAngleMap.get(angleDegrees);
  }

  public void setRightServoAngle(double angleDegrees){
    this.rightServoPos = rightServoAngleMap.get(angleDegrees);  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftHoodieServo.set(leftServoPos);
    rightHoodieServo.set(rightServoPos);

    hoodLeftSetPositionPublisher.set(leftServoPos);
    hoodLeftPositionPublisher.set(leftHoodieServo.getPosition());
    hoodRightSetPositionPublisher.set(rightServoPos);
    hoodRightPositionPublisher.set(rightHoodieServo.getPosition());

  }
}
