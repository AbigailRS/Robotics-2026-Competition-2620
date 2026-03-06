// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldZoneManager;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoods;
import frc.robot.subsystems.rockDestroyerInxder;
import frc.robot.subsystems.tinyPebbleShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Pass extends Command {
  /** Creates a new Shoot. */
  tinyPebbleShooter shooter;
  rockDestroyerInxder indexer;
  boolean leftSpeedReached, rightSpeedReached;
  CommandSwerveDrivetrain drivetrain;

  public Pass(tinyPebbleShooter shooter, rockDestroyerInxder indexer, CommandSwerveDrivetrain drivetrain) {
    this.indexer = indexer;
    this.shooter = shooter;
    this.drivetrain = drivetrain;

    addRequirements(indexer, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setLeftSlingVelocity(Constants.SHOOTER_PASS_VELOCITY);
    shooter.setRightSlingVelocity(Constants.SHOOTER_PASS_VELOCITY);

    indexer.setConveryVoltage(Constants.CONVEYOR_VOLTAGE_PERCENTAGE);

    if (shooter.atLeftShootVelocity()) {
      leftSpeedReached = true; 
    }

    if (shooter.atRightShootVelocity()) {
      rightSpeedReached = true; 
    }

    if(leftSpeedReached && FieldZoneManager.inCenterY(drivetrain.getState().Pose.getTranslation())){
      indexer.setLeftRockSumusherVoltage(Constants.LEFT_ROCK_SMUSHER_VOLTAGE);
    }
    else{
      indexer.setLeftRockSumusherVoltage(0.0);
    }
    if(rightSpeedReached && FieldZoneManager.inCenterY(drivetrain.getState().Pose.getTranslation())){
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
