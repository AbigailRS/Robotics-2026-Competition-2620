// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoods;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.rockDestroyerInxder;
import frc.robot.subsystems.tinyPebbleShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootV2 extends Command {
  // Subsystems
  private final Turret turretSubsystem;
  private final Hoods hoodSubsystem;
  private final tinyPebbleShooter flywheelSubsystem;
  private final CommandSwerveDrivetrain driveTrain;

  // Tuned Constants
  double totalExitVelocity = 55.0; // m/s
  Translation2d goalLocation;
  /**
   * Maps Distance to RPM
   */
  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  public ShootV2(Turret turret, Hoods hood, tinyPebbleShooter flyWheel, CommandSwerveDrivetrain driveTrain)
  {
    turretSubsystem = turret;
    hoodSubsystem = hood;
    flywheelSubsystem = flyWheel;
    this.driveTrain = driveTrain;
    addRequirements(turret, hood, flyWheel);
  }

  @Override
  public void initialize()
  {
    shooterTable.put(0.5, 37.0);
    shooterTable.put(1.0, 42.0);
    shooterTable.put(2.0, 52.0);
    shooterTable.put(3.0, 59.5);
    shooterTable.put(4.0, 64.0);
    shooterTable.put(5.0, 70.0);

  }

  @Override
  public void execute()
  {
    var robotSpeed = driveTrain.getState().Speeds;
    // 1. LATENCY COMP
    Translation2d futurePos = driveTrain.getState().Pose.getTranslation().plus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(Constants.LATENCY_CONSTANT)
                                                                   );

    // 2. GET TARGET VECTOR
    if(DriverStation.getAlliance().get() == Alliance.Red){
      goalLocation = Constants.POSE_RED_HUB;
    }
    else{
      goalLocation = Constants.POSE_BLUE_HUB;
    }
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = shooterTable.get(dist);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    double turretAngle        = shotVec.getAngle().getDegrees();
    double newHorizontalSpeed = shotVec.getNorm();

    // 6. SOLVE FOR NEW PITCH/RPM
    // Assuming constant total exit velocity, variable hood:
    // Clamp to avoid domain errors if we need more speed than possible
    double ratio    = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
    double newPitch = Math.acos(ratio);

    // 7. SET OUTPUTS
    turretSubsystem.setTurretPosition(turretAngle);
    hoodSubsystem.setLeftServoAngle(newPitch);
    hoodSubsystem.setRightServoAngle(newPitch);
    flywheelSubsystem.setLeftSlingVelocity(shooterTable.get(shotVec.getNorm()));
    flywheelSubsystem.setRightSlingVelocity(shooterTable.get(shotVec.getNorm()));
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {

  }

}
