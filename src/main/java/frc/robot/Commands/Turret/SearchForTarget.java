// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SearchForTarget extends Command {
  /** Creates a new SearchForTarget. */
  boolean leftPhase = true;
  Turret turret;


  public SearchForTarget(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(leftPhase){
      turret.setTurretVoltage(-Constants.TURRET_SEARCH_VOLTAGE);
    }
    else{
      turret.setTurretVoltage(Constants.TURRET_SEARCH_VOLTAGE);
    }

    if(turret.isStalled()){
      if(leftPhase){
        leftPhase = false;
      }
      else{
        leftPhase = true;
      }
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
