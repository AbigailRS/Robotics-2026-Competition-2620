// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.tinyPebbleShooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class rightSlingVelocity extends Command {

  tinyPebbleShooter pebbleShooter;
  /** Creates a new leftSlingVelocity. */
  public rightSlingVelocity(tinyPebbleShooter pebbleShooter) {
    this.pebbleShooter = pebbleShooter;
    addRequirements(pebbleShooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pebbleShooter.setRightSlingVelocity(Constants.VELOCITY_RIGHT_SLING);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pebbleShooter.setRightSlingVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
