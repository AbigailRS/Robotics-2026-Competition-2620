// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Turret;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldZoneManager;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoods;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SearchForTargetV2_SOM extends Command {
  /** Creates a new SearchForTarget. */
  Turret turret;
  CommandSwerveDrivetrain driveTrain;

  double deltaX, deltaY, angleToGoalDegrees;
  int remainder;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Turret Search V2");
  private final DoublePublisher rawAngleToGoalPub = table.getDoubleTopic("Raw Angle To Goal").publish(),
                                adjustedAngleToGoalPub = table.getDoubleTopic("Adjusted Angle To Goal").publish(),
                                pigeonAnglePub = table.getDoubleTopic("Pigeon Angle").publish(),
                                projectedPosX = table.getDoubleTopic("Proj X").publish(),
                                projectedPosY = table.getDoubleTopic("Proj Y").publish();

                                

  public SearchForTargetV2_SOM(Turret turret, CommandSwerveDrivetrain driveTrain) {
    this.turret = turret;
    this.driveTrain = driveTrain;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double latency = 0.5;

    Translation2d projectedPosition = driveTrain.getState().Pose.getTranslation().plus(
      new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond).times(latency));

    Translation2d projectedGoalLocationOffset = driveTrain.getState().Pose.getTranslation().minus(projectedPosition);

    projectedPosX.set(projectedGoalLocationOffset.getX());
    projectedPosY.set(projectedGoalLocationOffset.getY());

    if(DriverStation.getAlliance().get() == Alliance.Blue){
      projectedGoalLocationOffset = Constants.POSE_BLUE_HUB.minus(projectedGoalLocationOffset);
      deltaX = projectedGoalLocationOffset.getX() - driveTrain.getState().Pose.getX();
      deltaY = projectedGoalLocationOffset.getY() - driveTrain.getState().Pose.getY();
    }
    else{
      projectedGoalLocationOffset = Constants.POSE_RED_HUB.minus(projectedGoalLocationOffset);
      deltaX = projectedGoalLocationOffset.getX() - driveTrain.getState().Pose.getX();
      deltaY = projectedGoalLocationOffset.getY() - driveTrain.getState().Pose.getY();
    }
    
    angleToGoalDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));
    rawAngleToGoalPub.set(angleToGoalDegrees);
    angleToGoalDegrees = angleToGoalDegrees - driveTrain.getState().Pose.getRotation().getDegrees();

    if(angleToGoalDegrees > 180){
      angleToGoalDegrees = angleToGoalDegrees - 360;
    }
    else if(angleToGoalDegrees < -180){
      angleToGoalDegrees = angleToGoalDegrees + 360;
    }
    

    adjustedAngleToGoalPub.set(angleToGoalDegrees);
    pigeonAnglePub.set(driveTrain.getState().Pose.getRotation().getDegrees());
    if(turret.manualRotateEnabled()){
      turret.setTurretVoltage(0);
    }
    else{
      turret.setTurretPosition(angleToGoalDegrees);
    }

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
