// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackHub extends Command {
  /** Creates a new TrackTarget. */

  CommandSwerveDrivetrain drivetrain;
  Turret turret;
  double voltage;

  PIDController rotateTurretPIDController;
  

  public TrackHub(CommandSwerveDrivetrain drivetrain, Turret turret) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateTurretPIDController = new PIDController(Constants.TURRET_P, Constants.TURRET_I, Constants.TURRET_D);
    voltage = 0.0;
    LimelightHelpers.SetThrottle(Constants.PRIMARY_LL_NAME, 0);
    LimelightHelpers.SetThrottle(Constants.SECONDARY_LL_NAME, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.updateTargetTags();
    if(turret.priLLHasTarget()){
      voltage = rotateTurretPIDController.calculate(LimelightHelpers.getTY(Constants.PRIMARY_LL_NAME), 0);
      System.out.print("Pri has Target: " + voltage);
    }
    else if(turret.secLLHasTarget()){
      voltage = rotateTurretPIDController.calculate(LimelightHelpers.getTY(Constants.SECONDARY_LL_NAME), 0);
      System.out.print("Sec has Target: " + voltage);
    }
    else{
      //Add code for pose based aiming
    }
    turret.setTurretVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.setTurretVoltage(0);
    // LimelightHelpers.SetThrottle(Constants.PRIMARY_LL_NAME, 200);
    // LimelightHelpers.SetThrottle(Constants.SECONDARY_LL_NAME, 200);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
