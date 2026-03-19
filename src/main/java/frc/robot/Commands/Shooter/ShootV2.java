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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
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
  private InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<Double, FullShooterParams>(InverseInterpolator.forDouble(), null);
  public record FullShooterParams(double rps, double hoodAngle, double tof) {}

  private InterpolatingDoubleTreeMap inverseShooterMap = new InterpolatingDoubleTreeMap();
  private Translation2d goalPosition;

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
    SHOOTER_MAP.put(1.5, new FullShooterParams(0.45, 35.0, 0.38));
    SHOOTER_MAP.put(2.0, new FullShooterParams(0.5, 38.0, 0.45));
    SHOOTER_MAP.put(2.5, new FullShooterParams(0.55, 42.0, 0.52));
    SHOOTER_MAP.put(3.0, new FullShooterParams(0.60, 46.0, 0.60));
    SHOOTER_MAP.put(3.5, new FullShooterParams(0.65, 50.0, 0.68));
    SHOOTER_MAP.put(4.0, new FullShooterParams(0.7, 54.0, 0.76));
    SHOOTER_MAP.put(4.5, new FullShooterParams(0.75, 58.0, 0.85));
    SHOOTER_MAP.put(5.0, new FullShooterParams(0.8, 62.0, 0.94));

    inverseShooterMap.put(0.45, 1.5);
    inverseShooterMap.put(0.50, 2.0);
    inverseShooterMap.put(0.55, 2.5);
    inverseShooterMap.put(0.60, 3.0);
    inverseShooterMap.put(0.65, 3.5);
    inverseShooterMap.put(0.70, 4.0);
    inverseShooterMap.put(0.75, 4.5);
    inverseShooterMap.put(0.80, 5.0);
  }

  @Override
  public void execute()
  {

    // 1. Project future position
    Translation2d futurePos = driveTrain.getState().Pose.getTranslation().plus(
      new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond).times(Constants.LATENCY_CONSTANT)
    );

    // 2. Get target vector
    if(DriverStation.getAlliance().get() == Alliance.Red){
      goalPosition = Constants.POSE_RED_HUB;
    }
    else{
      goalPosition = Constants.POSE_BLUE_HUB;
    }
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);

    // 3. Look up baseline velocity from table
    FullShooterParams baseline = SHOOTER_MAP.get(distance);
    double baselineVelocity = distance / baseline.tof;

    // 4. Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // 5. THE MAGIC: subtract robot velocity
    Translation2d shotVelocity = targetVelocity.minus(new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond));

    // 6. Extract results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();

    // 7. Use table in reverse: velocity → effective distance → RPM
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
    double requiredRpm = SHOOTER_MAP.get(effectiveDistance).rps;

    double velocityRatio = requiredVelocity / baselineVelocity;

    // Split the correction: sqrt gives equal "contribution" from each
    double rpsFactor = Math.sqrt(velocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);

    // Apply RPM scaling
    double adjustedRps = baseline.rps * rpsFactor;

    // Apply hood adjustment (changes horizontal component)
    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle));
    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    turretSubsystem.setTurretPosition(turretAngle.getDegrees());
    hoodSubsystem.setLeftServoAngle(adjustedHood);
    hoodSubsystem.setRightServoAngle(adjustedHood);
    flywheelSubsystem.setLeftSlingVelocity(adjustedRps);
    flywheelSubsystem.setRightSlingVelocity(adjustedRps);


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

  public double getHorizontalVelocity(double distance) {
    FullShooterParams params = SHOOTER_MAP.get(distance);
    return distance / params.tof;
  }

  public double velocityToEffectiveDistance(double velocity) {
    return inverseShooterMap.get(velocity);
  }

  public double calculateAdjustedRpm(double requiredVelocity) {
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
    return SHOOTER_MAP.get(effectiveDistance).rps;
  }

}
