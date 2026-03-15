// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.AutoWait;
import frc.robot.Commands.SpeedUpdater;
import frc.robot.Commands.UpdateGameState;
import frc.robot.Commands.climbOff;
import frc.robot.Commands.climbOn;
import frc.robot.Commands.Cameras.EnableCamera;
import frc.robot.Commands.Conveyor.LeftDown;
import frc.robot.Commands.Conveyor.LeftUp;
import frc.robot.Commands.Conveyor.RightDown;
import frc.robot.Commands.Conveyor.RightUp;
import frc.robot.Commands.Conveyor.converybackwards;
import frc.robot.Commands.Conveyor.converyforword;
import frc.robot.Commands.Hood.RetractHoods;
import frc.robot.Commands.Hood.SetHoodForPass;
import frc.robot.Commands.Hood.SetHoodForShoot;
import frc.robot.Commands.Hood.TESTSetHoodsHigh;
import frc.robot.Commands.Hood.TESTSetHoodsLow;
import frc.robot.Commands.Intake.IntakeExtend;
import frc.robot.Commands.Intake.IntakeExtendPos;
import frc.robot.Commands.Intake.IntakeRetractPos;
import frc.robot.Commands.Intake.IntakeRetractShoot;
import frc.robot.Commands.Intake.ZeroIntake;
import frc.robot.Commands.Intake.IntakeRefund;
import frc.robot.Commands.Intake.IntakeRetract;
import frc.robot.Commands.Shooter.ManualShoot;
import frc.robot.Commands.Shooter.Pass;
import frc.robot.Commands.Shooter.Shoot;
import frc.robot.Commands.Shooter.leftSlingShot;
import frc.robot.Commands.Shooter.leftSlingVelocity;
import frc.robot.Commands.Shooter.rightSlingShot;
import frc.robot.Commands.Shooter.rightSlingVelocity;
import frc.robot.Commands.Turret.DisableManualRotate;
import frc.robot.Commands.Turret.ManualRotate;
import frc.robot.Commands.Turret.ResetTurretEncoder;
import frc.robot.Commands.Turret.SearchForTarget;
import frc.robot.Commands.Turret.SearchForTargetV2;
import frc.robot.Commands.Turret.SearchForTargetV2_SOM;
import frc.robot.Commands.Turret.TargetAllianceWall;
import frc.robot.Commands.Turret.TrackHub;
import frc.robot.Commands.Turret.TrackHub_SOM;
import frc.robot.Commands.Turret.ZeroTurret;
import frc.robot.enums.GameState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoods;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.bigRockIntake;
import frc.robot.subsystems.mountainClimber;
import frc.robot.subsystems.rockDestroyerInxder;
import frc.robot.subsystems.tinyPebbleShooter;




public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final tinyPebbleShooter shooter = new tinyPebbleShooter();
    public final bigRockIntake intake = new bigRockIntake();
    public final rockDestroyerInxder index = new rockDestroyerInxder();
    public final Turret turret = new Turret();
    public final Hoods hoods = new Hoods();
    public final mountainClimber climb = new mountainClimber();

    public final GameStateManager gameStateManager = new GameStateManager();

    public final CameraSystem photonCamera1 = new CameraSystem("Photon Camera 1", new Translation3d(Constants.CAMERA_1_TRANSLATION_X, Constants.CAMERA_1_TRANSLATION_Y, Constants.CAMERA_1_TRANSLATION_Z), 
                                                                                                new Rotation3d(Constants.CAMERA_1_ROTATION_ROLL, Constants.CAMERA_1_ROTATION_PITCH, Constants.CAMERA_1_ROTATION_YAW));

    public final CameraSystem photonCamera2 = new CameraSystem("Photon Camera 2", new Translation3d(Constants.CAMERA_2_TRANSLATION_X, Constants.CAMERA_2_TRANSLATION_Y, Constants.CAMERA_2_TRANSLATION_Z), 
                                                                                                new Rotation3d(Constants.CAMERA_2_ROTATION_ROLL, Constants.CAMERA_2_ROTATION_PITCH, Constants.CAMERA_2_ROTATION_YAW));
    
    public final CameraSystem photonCamera3 = new CameraSystem("Photon Camera 3", new Translation3d(Constants.CAMERA_3_TRANSLATION_X, Constants.CAMERA_3_TRANSLATION_Y, Constants.CAMERA_3_TRANSLATION_Z), 
                                                                                                new Rotation3d(Constants.CAMERA_3_ROTATION_ROLL, Constants.CAMERA_3_ROTATION_PITCH, Constants.CAMERA_3_ROTATION_YAW));
    
    public final CameraSystem photonCamera4 = new CameraSystem("Photon Camera 4", new Translation3d(Constants.CAMERA_4_TRANSLATION_X, Constants.CAMERA_4_TRANSLATION_Y, Constants.CAMERA_4_TRANSLATION_Z), 
                                                                                                new Rotation3d(Constants.CAMERA_4_ROTATION_ROLL, Constants.CAMERA_4_ROTATION_PITCH, Constants.CAMERA_4_ROTATION_YAW));
    
    private final SendableChooser<Command> autoChooser;

    private final Trigger updateGameState = new Trigger(() -> DriverStation.getMatchTime() == 139.0 ||
                                                            DriverStation.getMatchTime() == 129.0 ||
                                                            DriverStation.getMatchTime() == 104.0 ||
                                                            DriverStation.getMatchTime() == 79.0 ||
                                                            DriverStation.getMatchTime() == 54.0 ||
                                                            DriverStation.getMatchTime() == 29.0
                                                        );

    // private final Trigger noApriltagsForTurret = new Trigger(() -> turret.getTimeSinceLastSighted() > 0.2 && !turret.manualRotateEnabled() && DriverStation.isEnabled());
    // private final Trigger ApriltagsFoundForTurret = new Trigger(() -> (turret.priLLHasTarget() || turret.secLLHasTarget()) && !turret.manualRotateEnabled() && DriverStation.isEnabled());

    private final Trigger inOwnZoneTrigger = new Trigger(() -> FieldZoneManager.inOwnZoneX(drivetrain.getState().Pose.getTranslation()) && !DriverStation.isAutonomous());
    private final Trigger outOfZoneTrigger = new Trigger(() -> !FieldZoneManager.inOwnZoneX(drivetrain.getState().Pose.getTranslation()) && !DriverStation.isAutonomous());

    private final Trigger shootTrigger = new Trigger(() -> driver.rightTrigger().getAsBoolean() && FieldZoneManager.inOwnZoneX(drivetrain.getState().Pose.getTranslation()) && !turret.manualRotateEnabled() && !DriverStation.isDisabled() && Math.abs(driver.getLeftX()) < 0.1 && Math.abs(driver.getLeftY()) < 0.1 && !DriverStation.isAutonomous());
    private final Trigger passTrigger = new Trigger(() -> driver.rightTrigger().getAsBoolean() && !FieldZoneManager.inOwnZoneX(drivetrain.getState().Pose.getTranslation()) && !turret.manualRotateEnabled() && !DriverStation.isDisabled() && !DriverStation.isAutonomous());
    private final Trigger manualTrigger = new Trigger(() -> driver.rightTrigger().getAsBoolean() && turret.manualRotateEnabled() && !DriverStation.isDisabled() && !DriverStation.isAutonomous());
    private final Trigger shootOnMoveTrigger = new Trigger(() -> driver.rightTrigger().getAsBoolean() && FieldZoneManager.inOwnZoneX(drivetrain.getState().Pose.getTranslation()) && !turret.manualRotateEnabled() && !DriverStation.isDisabled() && (Math.abs(driver.getLeftX()) > 0.1 || Math.abs(driver.getLeftY()) > 0.1 && !DriverStation.isAutonomous()));
    // private final Trigger passOnMoveTrigger = new Trigger(() -> driver.rightBumper().getAsBoolean() && !FieldZoneManager.inOwnZoneX(drivetrain.getState().Pose.getTranslation()) && !turret.manualRotateEnabled());
    private final Trigger notAuto = new Trigger(() -> !DriverStation.isAutonomous());
    
    public RobotContainer() {

        NamedCommands.registerCommand("ZeroIntake", new ZeroIntake(intake));
        NamedCommands.registerCommand("ZeroTurret", new ZeroTurret(turret));
        NamedCommands.registerCommand("ShootOn", new Shoot(shooter, index, drivetrain));
        NamedCommands.registerCommand("ShootOff", new Shoot(shooter, index, drivetrain, 0.0));
        NamedCommands.registerCommand("ExtendIntake", new IntakeExtendPos(intake));
        NamedCommands.registerCommand("RetractIntake", new IntakeRetractPos(intake));
        NamedCommands.registerCommand("Wait3s", new AutoWait(3.0));
        NamedCommands.registerCommand("TrackHub", new SearchForTargetV2_SOM(turret, drivetrain));
        NamedCommands.registerCommand("IntakeRetractShoot", new IntakeRetractShoot(intake));
        NamedCommands.registerCommand("HoodsForShoot", new SetHoodForShoot(hoods, drivetrain));
        NamedCommands.registerCommand("RetractHoods", new RetractHoods(hoods));
        NamedCommands.registerCommand("CamsOn", new EnableCamera(photonCamera1, photonCamera2, photonCamera3, photonCamera4));
        NamedCommands.registerCommand("CamsOff", new EnableCamera(photonCamera1, photonCamera2, photonCamera3, photonCamera4));

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(ControllerModifier.modifyX(driver.getLeftY(), driver.rightTrigger().getAsBoolean()) * Constants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(ControllerModifier.modifyY(driver.getLeftX(), driver.rightTrigger().getAsBoolean()) * Constants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));  
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);

        updateGameState.onTrue(new UpdateGameState(gameStateManager));

        turret.setDefaultCommand(new SearchForTargetV2_SOM(turret, drivetrain));
        intake.setDefaultCommand(new IntakeRetractPos(intake));
        hoods.setDefaultCommand(new RetractHoods(hoods));

        // OPERATOR CONTROLS

        driver.y().whileTrue(new TargetAllianceWall(turret, drivetrain));
        //notAuto.whileTrue(new )
        // inOwnZoneTrigger.onTrue(new SearchForTargetV2_SOM(turret, drivetrain, hoods));
        outOfZoneTrigger.whileTrue(new TargetAllianceWall(turret, drivetrain));
        //driver.rightBumper().whileTrue(new ResetTurretEncoder(turret));

        shootTrigger.whileTrue(new Shoot(shooter, index, drivetrain));
        shootTrigger.whileTrue(new SetHoodForShoot(hoods, drivetrain));
        shootTrigger.whileTrue(drivetrain.applyRequest(() -> brake));
        shootTrigger.whileTrue(new IntakeRetractShoot(intake));
        driver.rightTrigger().onFalse(new IntakeExtendPos(intake));
        passTrigger.whileTrue(new Pass(shooter, index, drivetrain));
        passTrigger.whileTrue(new SetHoodForPass(hoods));
        manualTrigger.whileTrue(new ManualShoot(shooter, index));

        shootOnMoveTrigger.whileTrue(new Shoot(shooter, index, drivetrain));
        shootOnMoveTrigger.whileTrue(new SetHoodForShoot(hoods, drivetrain));
        shootOnMoveTrigger.whileTrue(new IntakeExtendPos(intake));
        //driver.leftBumper().whileTrue(new ResetTurretEncoder(turret));
        driver.leftBumper().onTrue(new ZeroIntake(intake));
        driver.leftBumper().onTrue(new ZeroTurret(turret));

        driver.povLeft().whileTrue(new ManualRotate(turret, 12.0));
        driver.povRight().whileTrue(new ManualRotate(turret, -12.0));
        driver.povDown().whileTrue(new DisableManualRotate(turret));
        //driver.povDown().whileTrue(new TESTSetHoodsLow(hoods));
        //driver.povUp().whileTrue(new TESTSetHoodsHigh(hoods));
        driver.x().toggleOnTrue(new IntakeExtendPos(intake));
        driver.a().whileTrue(new IntakeExtend(intake));
        driver.b().whileTrue(new IntakeRetract(intake));
    }

    public Command getAutonomousCommand() {
        //return Commands.none();
        return autoChooser.getSelected();
    }
}
