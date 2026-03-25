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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldZoneManager;
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
  private final rockDestroyerInxder indexer;

  // Tuned Constants
  double totalExitVelocity = 55.0; // m/s
  Translation2d goalLocation;

  private InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap inverseShooterMap = new InterpolatingDoubleTreeMap();
  private Translation2d goalPosition;

  private boolean leftSpeedReached, rightSpeedReached;
  private double timeout = -1.0;
  private Timer timeoutTimer = new Timer();
  private Timer shootDelayTimer = new Timer();

  private Transform2d leftTurretOffset, rightTurretOffset;
  private Pose2d leftTurretPose, rightTurretPose;

  private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private static final NetworkTable table = inst.getTable("Shoot V2");
  private static final DoublePublisher distToGoalPub = table.getDoubleTopic("distance to goal").publish();
  private static final DoublePublisher leftTurretDistToGoalPub = table.getDoubleTopic("Left Turret Distance to Goal").publish();
  private static final DoublePublisher rightTurretDistToGoalPub = table.getDoubleTopic("Left Turret Distance to Goal").publish();
  private static final DoublePublisher hoodAnglePub = table.getDoubleTopic("Hood Angle").publish();

  public ShootV2(Turret turret, Hoods hood, tinyPebbleShooter flyWheel, CommandSwerveDrivetrain driveTrain, rockDestroyerInxder indexer)
  {
    turretSubsystem = turret;
    hoodSubsystem = hood;
    flywheelSubsystem = flyWheel;
    this.driveTrain = driveTrain;
    this.indexer = indexer;
    addRequirements(turret, hood, flyWheel, indexer);
  }

    public ShootV2(Turret turret, Hoods hood, tinyPebbleShooter flyWheel, CommandSwerveDrivetrain driveTrain, rockDestroyerInxder indexer, double timeout)
  {
    turretSubsystem = turret;
    hoodSubsystem = hood;
    flywheelSubsystem = flyWheel;
    this.driveTrain = driveTrain;
    this.indexer = indexer;
    this.timeout = timeout;
    addRequirements(turret, hood, flyWheel, indexer);
  }

  @Override
  public void initialize()
  {
    if(timeout >= 0){
      timeoutTimer.reset();
      timeoutTimer.start();
    }

    shooterMap.put(1.5, 45.0);
    shooterMap.put(2.0, 50.0);
    shooterMap.put(2.5, 55.0);
    shooterMap.put(3.0, 57.0);
    shooterMap.put(3.5, 58.0);
    shooterMap.put(4.0, 60.0);
    shooterMap.put(4.5, 65.0);

    hoodAngleMap.put(5.0, 0.0);
    hoodAngleMap.put(4.5, 0.0);
    hoodAngleMap.put(4.0, 2.0);
    hoodAngleMap.put(3.5, 6.0);
    hoodAngleMap.put(3.0, 10.0);
    hoodAngleMap.put(2.5, 14.0);
    hoodAngleMap.put(2.0, 17.0);
    hoodAngleMap.put(1.5, 20.0);

    tofMap.put(1.5, 0.85);
    tofMap.put(2.0, 1.0);
    tofMap.put(2.5, 1.25);
    tofMap.put(3.0, 1.3);
    tofMap.put(3.5, 1.1);
    tofMap.put(4.0, 1.0);
    tofMap.put(4.5, 1.2);

    inverseShooterMap.put(45.0, 1.5);
    inverseShooterMap.put(50.0, 2.0);
    inverseShooterMap.put(55.0, 2.5);
    inverseShooterMap.put(57.0, 3.0);
    inverseShooterMap.put(58.0, 3.5);
    inverseShooterMap.put(60.0, 4.0);
    inverseShooterMap.put(63.0, 4.5);
    inverseShooterMap.put(65.0, 5.0);

    leftSpeedReached = false;
    rightSpeedReached = false;
  }

  @Override
  public void execute()
  {

    leftTurretOffset = new Transform2d(Constants.TURRET_LEFT_ROBOT_OFFSET_X, Constants.TURRET_LEFT_ROBOT_OFFSET_Y, new Rotation2d());
    rightTurretOffset = new Transform2d(Constants.TURRET_RIGHT_ROBOT_OFFSET_X, Constants.TURRET_RIGHT_ROBOT_OFFSET_Y, new Rotation2d());

    leftTurretPose = driveTrain.getState().Pose.plus(leftTurretOffset);
    // 1. Project future position
    Translation2d futurePos = driveTrain.getState().Pose.getTranslation().plus(
      new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond).times(Constants.LATENCY_CONSTANT)
    );
    Translation2d futurePosLeftTurret = leftTurretPose.getTranslation().plus(
      new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond).times(Constants.LATENCY_CONSTANT)
    );
    Translation2d futurePosRightTurret = leftTurretPose.getTranslation().plus(
      new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond).times(Constants.LATENCY_CONSTANT)
    );
    

    leftTurretOffset = new Transform2d(Constants.TURRET_LEFT_ROBOT_OFFSET_X, Constants.TURRET_LEFT_ROBOT_OFFSET_Y, new Rotation2d());
    rightTurretOffset = new Transform2d(Constants.TURRET_RIGHT_ROBOT_OFFSET_X, Constants.TURRET_RIGHT_ROBOT_OFFSET_Y, new Rotation2d());

    // 2. Get target vector
    if(DriverStation.getAlliance().get() == Alliance.Red){
      goalPosition = Constants.POSE_RED_HUB;
    }
    else{
      goalPosition = Constants.POSE_BLUE_HUB;
    }
    Translation2d toGoal = goalPosition.minus(futurePos);
    Translation2d leftTurretToGoal = goalPosition.minus(futurePosLeftTurret);
    Translation2d rightTurretToGoal = goalPosition.minus(futurePosRightTurret);
    double distance = toGoal.getNorm();
    double leftTurretDistance = leftTurretToGoal.getNorm();
    double rightTurretDistance = rightTurretToGoal.getNorm();
    Translation2d targetDirection = toGoal.div(distance);

    // 3. Look up baseline velocity from table
    double shooterBaseline = shooterMap.get(distance);
    double leftShooterBaseline = shooterMap.get(leftTurretDistance);
    double rightShooterBaseline = shooterMap.get(rightTurretDistance);
    double hoodBaseline = hoodAngleMap.get(distance);
    double leftHoodBaseline = hoodAngleMap.get(leftTurretDistance);
    double rightHoodBaseline = hoodAngleMap.get(rightTurretDistance);
    double tofBaseline = tofMap.get(distance);
    double leftTofBaseline = tofMap.get(leftTurretDistance);
    double rightTofBaseline = tofMap.get(rightTurretDistance);
    double baselineVelocity = distance / tofBaseline;
    double leftBaselineVelocity = leftTurretDistance / leftTofBaseline;
    double rightBaselineVelocity = rightTurretDistance / rightTofBaseline;

    // 4. Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);
    Translation2d leftTargetVelocity = targetDirection.times(leftBaselineVelocity);
    Translation2d rightTargetVelocity = targetDirection.times(rightBaselineVelocity);

    // 5. THE MAGIC: subtract robot velocity
    Translation2d shotVelocity = targetVelocity.minus(new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond));
    Translation2d leftShotVelocity = leftTargetVelocity.minus(new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond));
    Translation2d rightShotVelocity = rightTargetVelocity.minus(new Translation2d(driveTrain.getState().Speeds.vxMetersPerSecond, driveTrain.getState().Speeds.vyMetersPerSecond));

    // 6. Extract results
    Rotation2d turretAngle = shotVelocity.getAngle();
    double requiredVelocity = shotVelocity.getNorm();
    double leftRequiredVelocity = leftShotVelocity.getNorm();
    double rightRequiredVelocity = rightShotVelocity.getNorm();

    // 7. Use table in reverse: velocity → effective distance → RPM
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
    double leftEffectiveDistance = velocityToEffectiveDistance(leftRequiredVelocity);
    double rightEffectiveDistance = velocityToEffectiveDistance(rightRequiredVelocity);
    double requiredRpm = shooterMap.get(effectiveDistance);
    double leftRequiredRpm = shooterMap.get(leftEffectiveDistance);
    double rightRequiredRpm = shooterMap.get(rightEffectiveDistance);

    double velocityRatio = requiredVelocity / baselineVelocity;
    double leftVelocityRatio = leftRequiredVelocity / leftBaselineVelocity;
    double rightVelocityRatio = rightRequiredVelocity / rightBaselineVelocity;

    // Split the correction: sqrt gives equal "contribution" from each
    double rpsFactor = Math.sqrt(velocityRatio);
    double leftRpsFactor = Math.sqrt(leftVelocityRatio);
    double rightRpsFactor = Math.sqrt(rightVelocityRatio);
    double hoodFactor = Math.sqrt(velocityRatio);
    double leftHoodFactor = Math.sqrt(leftVelocityRatio);
    double rightHoodFactor = Math.sqrt(rightVelocityRatio);

    // Apply RPM scaling
    double adjustedRps = shooterBaseline * rpsFactor;
    double leftAdjustedRps = leftShooterBaseline * leftRpsFactor;
    double rightAdjustedRps = rightShooterBaseline * rightRpsFactor;

    // Apply hood adjustment (changes horizontal component)
    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(hoodBaseline));
    double leftTotalVelocity = leftBaselineVelocity / Math.cos(Math.toRadians(leftHoodBaseline));
    double rightTotalVelocity = rightBaselineVelocity / Math.cos(Math.toRadians(rightHoodBaseline));
    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double leftTargetHorizFromHood = leftBaselineVelocity * leftHoodFactor;
    double rightTargetHorizFromHood = rightBaselineVelocity * rightHoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double leftRatio = MathUtil.clamp(leftTargetHorizFromHood / leftTotalVelocity, 0.0, 1.0);
    double rightRatio = MathUtil.clamp(rightTargetHorizFromHood / rightTotalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));
    double leftAdjustedHood = Math.toDegrees(Math.acos(leftRatio));
    double rightAdjustedHood = Math.toDegrees(Math.acos(rightRatio));

    double angleToGoalDegrees = turretAngle.getDegrees() - driveTrain.getState().Pose.getRotation().getDegrees();
    if(angleToGoalDegrees > 180){
      angleToGoalDegrees = angleToGoalDegrees - 360;
    }
    else if(angleToGoalDegrees < -180){
      angleToGoalDegrees = angleToGoalDegrees + 360;
    }
    distToGoalPub.set(distance);
    leftTurretDistToGoalPub.set(leftTurretDistance);
    rightTurretDistToGoalPub.set(rightTurretDistance);
    hoodAnglePub.set(adjustedHood);


    turretSubsystem.setTurretPosition(angleToGoalDegrees);
    hoodSubsystem.setLeftServoAngle(leftAdjustedHood);
    hoodSubsystem.setRightServoAngle(rightAdjustedHood);
    flywheelSubsystem.setLeftSlingVelocity(leftAdjustedRps);
    flywheelSubsystem.setRightSlingVelocity(rightAdjustedRps);

    if(flywheelSubsystem.atLeftShootVelocity() || flywheelSubsystem.atRightShootVelocity()){
      shootDelayTimer.start();
      if(shootDelayTimer.get() > 1.0){
        indexer.setConveryVoltage(Constants.CONVEYOR_VOLTAGE_PERCENTAGE);
      }
    }
    else{
      indexer.setConveryVoltage(0.0);
    }

    if (flywheelSubsystem.atLeftShootVelocity()) {
      leftSpeedReached = true; 
    }

    if (flywheelSubsystem.atRightShootVelocity()) {
      rightSpeedReached = true; 
    }

    if(leftSpeedReached){
      indexer.setLeftRockSumusherVoltage(Constants.LEFT_ROCK_SMUSHER_VOLTAGE_PERCENTAGE);
    }
    else{
      indexer.setLeftRockSumusherVoltage(0.0);
    }
    if(rightSpeedReached){
      indexer.setRightRockSmusherVoltage(Constants.RIGHT_ROCK_SMUSHER_VOLTAGE_PERCENTAGE);
    }
    else{
      indexer.setRightRockSmusherVoltage(0.0);
    }

  }

  @Override
  public boolean isFinished()
  {
    if(timeout >= 0){
      if(timeoutTimer.get() > timeout){
        return true;
      }
    }
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    turretSubsystem.setTurretPosition(0);
    hoodSubsystem.setLeftServoPosition(0);
    hoodSubsystem.setLeftServoPosition(0);
    flywheelSubsystem.setLeftSlingVelocity(0);
    flywheelSubsystem.setRightSlingVelocity(0);
    indexer.setLeftRockSumusherVoltage(0.0);
    indexer.setRightRockSmusherVoltage(0.0);
    indexer.setConveryVoltage(0);
    shootDelayTimer.reset();
  }

  public double getHorizontalVelocity(double distance) {
    double params = tofMap.get(distance);
    return distance / params;
  }

  public double velocityToEffectiveDistance(double velocity) {
    return inverseShooterMap.get(velocity);
  }

  // public double calculateAdjustedRpm(double requiredVelocity) {
  //   double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
  //   return SHOOTER_MAP.get(effectiveDistance).rps;
  // }

}
