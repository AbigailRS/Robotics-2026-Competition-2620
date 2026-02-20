// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.rockDestroyerInxder;
import frc.robot.subsystems.tinyPebbleShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new Shoot. */
  tinyPebbleShooter shooter;
  rockDestroyerInxder indexer;
  boolean leftSpeedReached, rightSpeedReached;

  InterpolatingDoubleTreeMap velocityIPMap = new InterpolatingDoubleTreeMap();


  public Shoot(tinyPebbleShooter shooter, rockDestroyerInxder indexer) {
    this.indexer = indexer;
    this.shooter = shooter;
    addRequirements(indexer, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leftSpeedReached = false;
    rightSpeedReached = false;

    velocityIPMap.put(0.5, 90.0);
    velocityIPMap.put(1.0, 92.0);
    velocityIPMap.put(2.0, 93.0);
    velocityIPMap.put(3.0, 93.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setLeftSlingVelocity(Constants.VELOCITY_LEFT_SLING);
    shooter.setRightSlingVelocity(Constants.VELOCITY_RIGHT_SLING);
    indexer.setConveryVoltage(Constants.CONVEYOR_VOLTAGE_PERCENTAGE);

    if (shooter.atLeftShootVelocity()) {
      leftSpeedReached = true; 
    }

    if (shooter.atRightShootVelocity()) {
      rightSpeedReached = true; 
    }

    if(leftSpeedReached){
      indexer.setLeftRockSumusherVoltage(Constants.LEFT_ROCK_SMUSHER_VOLTAGE);
    }
    else{
      indexer.setLeftRockSumusherVoltage(0.0);
    }
    if(rightSpeedReached){
      indexer.setRightRockSmusherVoltage(Constants.RIGHT_ROCK_SMUSHER_VOLTAGE);
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
    return false;
  }
}
