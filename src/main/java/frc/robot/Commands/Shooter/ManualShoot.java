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
public class ManualShoot extends Command {
  /** Creates a new Shoot. */
  tinyPebbleShooter shooter;
  rockDestroyerInxder indexer;
  CommandSwerveDrivetrain drivetrain;
  Timer shootDelayTimer;

  public ManualShoot(tinyPebbleShooter shooter, rockDestroyerInxder indexer) {
    this.indexer = indexer;
    this.shooter = shooter;

    //addRequirements(indexer, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootDelayTimer = new Timer();
    shootDelayTimer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setLeftSlingShotVoltage(Constants.SHOOTER_MANUAL_SHOOT_PERCENTAGE);
    shooter.setRightSlingShotVoltage(Constants.SHOOTER_MANUAL_SHOOT_PERCENTAGE);

    if (shootDelayTimer.get() > 1.0) {
      indexer.setRightRockSmusherVoltage(Constants.RIGHT_ROCK_SMUSHER_VOLTAGE);
      indexer.setLeftRockSumusherVoltage(Constants.LEFT_ROCK_SMUSHER_VOLTAGE);
    }
    else{
      indexer.setLeftRockSumusherVoltage(0.0);
      indexer.setRightRockSmusherVoltage(0.0);
    }
    if(shootDelayTimer.get() > 2.0){
      indexer.setConveryVoltage(Constants.CONVEYOR_VOLTAGE_PERCENTAGE);
    }
    else{
      indexer.setConveryVoltage(0.0);
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
    shootDelayTimer.reset();
    shootDelayTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
