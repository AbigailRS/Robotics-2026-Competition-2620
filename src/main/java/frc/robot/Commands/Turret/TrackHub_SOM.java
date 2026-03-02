// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldZoneManager;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoods;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackHub_SOM extends Command {
  /** Creates a new TrackTarget. */

  CommandSwerveDrivetrain drivetrain;
  Turret turret;
  double voltage;
  Hoods hoods;

  PIDController rotateTurretPIDController;

  InterpolatingDoubleTreeMap leftHoodIMap = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap rightHoodIMap = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap xSpeedMap = new InterpolatingDoubleTreeMap();
  InterpolatingDoubleTreeMap ySpeedMap = new InterpolatingDoubleTreeMap();

  public TrackHub_SOM(CommandSwerveDrivetrain drivetrain, Turret turret, Hoods hoods) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.hoods = hoods;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateTurretPIDController = new PIDController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D);
    voltage = 0.0;

    leftHoodIMap.put(1.0, 0.01);
    leftHoodIMap.put(2.0, 0.05);
    leftHoodIMap.put(3.0, 0.05);
    leftHoodIMap.put(5.0, 0.2);

    rightHoodIMap.put(1.0, 0.99);
    rightHoodIMap.put(2.0, 0.95);
    rightHoodIMap.put(3.0, 0.95);
    rightHoodIMap.put(5.0, 0.8);

    xSpeedMap.put(-0.5, -0.25);
    xSpeedMap.put(-1.0, -1.0);
    xSpeedMap.put(-1.5, -2.0);
    xSpeedMap.put(0.5, 0.25);
    xSpeedMap.put(1.0, 1.0);
    xSpeedMap.put(1.5, 2.0);

    ySpeedMap.put(-0.5, -0.25);
    ySpeedMap.put(-1.0, -1.0);
    ySpeedMap.put(-1.5, -2.0);
    ySpeedMap.put(0.5, 0.25);
    ySpeedMap.put(1.0, 1.0);
    ySpeedMap.put(1.5, 2.0);

    turret.updateTargetTags();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // LimelightHelpers.setFiducial3DOffset(Constants.PRIMARY_LL_NAME, xSpeedMap.get(drivetrain.getSpeedx()) - 0.5, ySpeedMap.get(drivetrain.getSpeedy()), 0.0);
    // LimelightHelpers.setFiducial3DOffset(Constants.SECONDARY_LL_NAME, xSpeedMap.get(drivetrain.getSpeedx()) - 0.5, ySpeedMap.get(drivetrain.getSpeedy()), 0.0);
    double[] offsetArray = {xSpeedMap.get(drivetrain.getSpeedx()) - 0.5, ySpeedMap.get(drivetrain.getSpeedy()), 0.0};
    NetworkTableInstance.getDefault().getTable(Constants.PRIMARY_LL_NAME).getEntry("fiducial_offset_set").setDoubleArray(offsetArray);
    NetworkTableInstance.getDefault().getTable(Constants.SECONDARY_LL_NAME).getEntry("fiducial_offset_set").setDoubleArray(offsetArray);


    if(turret.priLLHasTarget()){
      voltage = rotateTurretPIDController.calculate(LimelightHelpers.getTY(Constants.PRIMARY_LL_NAME), 0);
    }
    else if(turret.secLLHasTarget()){
      voltage = rotateTurretPIDController.calculate(-LimelightHelpers.getTY(Constants.SECONDARY_LL_NAME), 0);
    }
    else{
      //Add code for pose based aiming
    }
    turret.setTurretVoltage(voltage);
    hoods.setLeftServoPosition(leftHoodIMap.get(FieldZoneManager.getDistanceTogoal(drivetrain.getState().Pose.getTranslation())));
    hoods.setRightServoPosition(rightHoodIMap.get(FieldZoneManager.getDistanceTogoal(drivetrain.getState().Pose.getTranslation())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurretVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
