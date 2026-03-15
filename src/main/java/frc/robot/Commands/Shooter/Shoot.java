// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldZoneManager;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoods;
import frc.robot.subsystems.rockDestroyerInxder;
import frc.robot.subsystems.tinyPebbleShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new Shoot. */
  tinyPebbleShooter shooter;
  rockDestroyerInxder indexer;
  boolean leftSpeedReached, rightSpeedReached;
  CommandSwerveDrivetrain drivetrain;
  Timer timeoutTimer;
  double timeoutTime = -1;

  InterpolatingDoubleTreeMap velocityIPMap = new InterpolatingDoubleTreeMap();


  public Shoot(tinyPebbleShooter shooter, rockDestroyerInxder indexer, CommandSwerveDrivetrain drivetrain) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.drivetrain = drivetrain;

    addRequirements(indexer, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Shoot(tinyPebbleShooter shooter, rockDestroyerInxder indexer, CommandSwerveDrivetrain drivetrain, double timeoutTime) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.timeoutTime = timeoutTime;

    addRequirements(indexer, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftSpeedReached = false;
    rightSpeedReached = false;
    if(timeoutTime >= 0){
      timeoutTimer = new Timer();
      timeoutTimer.reset();
      timeoutTimer.start();
    }

    velocityIPMap.put(0.5, 37.0);
    velocityIPMap.put(1.0, 42.0);
    velocityIPMap.put(2.0, 52.0);
    velocityIPMap.put(3.0, 59.5);
    velocityIPMap.put(4.0, 64.0);
    velocityIPMap.put(5.0, 70.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    shooter.setLeftSlingVelocity(velocityIPMap.get(FieldZoneManager.getDistanceTogoal(drivetrain.getState().Pose.getTranslation())));
    shooter.setRightSlingVelocity(velocityIPMap.get(FieldZoneManager.getDistanceTogoal(drivetrain.getState().Pose.getTranslation())));
    indexer.setConveryVoltage(Constants.CONVEYOR_VOLTAGE_PERCENTAGE);

    if (shooter.atLeftShootVelocity()) {
      leftSpeedReached = true; 
    }

    if (leftSpeedReached) {
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


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setLeftSlingShotVoltage(0);
    shooter.setRightSlingShotVoltage(0);
    indexer.setConveryVoltage(0.0);
    indexer.setLeftRockSumusherVoltage(0.0);
    indexer.setRightRockSmusherVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timeoutTime >= 0){
      if(timeoutTimer.get() > timeoutTime){
        return true;
      }
    }
    return false;
  }
}
