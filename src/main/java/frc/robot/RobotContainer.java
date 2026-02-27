// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.SpeedUpdater;
import frc.robot.Commands.UpdateGameState;
import frc.robot.Commands.Conveyor.LeftDown;
import frc.robot.Commands.Conveyor.LeftUp;
import frc.robot.Commands.Conveyor.RightDown;
import frc.robot.Commands.Conveyor.RightUp;
import frc.robot.Commands.Conveyor.converybackwards;
import frc.robot.Commands.Conveyor.converyforword;
import frc.robot.Commands.Hood.TESTSetHoodsHigh;
import frc.robot.Commands.Hood.TESTSetHoodsLow;
import frc.robot.Commands.Intake.IntakeExtend;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.IntakeRefund;
import frc.robot.Commands.Intake.IntakeRetract;
import frc.robot.Commands.Shooter.Shoot;
import frc.robot.Commands.Shooter.leftSlingShot;
import frc.robot.Commands.Shooter.leftSlingVelocity;
import frc.robot.Commands.Shooter.rightSlingShot;
import frc.robot.Commands.Shooter.rightSlingVelocity;
import frc.robot.Commands.Turret.ManualRotate;
import frc.robot.Commands.Turret.SearchForTarget;
import frc.robot.Commands.Turret.TrackHub;
import frc.robot.enums.GameState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hoods;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.bigRockIntake;
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

    private final Trigger inOwnZone = new Trigger(() -> FieldZoneManager.inOwnZone(drivetrain.getState().Pose.getX()));

    private final Trigger updateGameState = new Trigger(() -> DriverStation.getMatchTime() == 139.0 ||
                                                            DriverStation.getMatchTime() == 129.0 ||
                                                            DriverStation.getMatchTime() == 104.0 ||
                                                            DriverStation.getMatchTime() == 79.0 ||
                                                            DriverStation.getMatchTime() == 54.0 ||
                                                            DriverStation.getMatchTime() == 29.0
                                                        );

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Start Loading Side Neutral Zone Climb");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(FieldZoneManager.inOwnZone(drivetrain.getState().Pose.getX()) ? -driver.getLeftY() * Constants.MaxSpeed : -driver.getLeftY() * Constants.MaxSpeed * Constants.SLOW_SPEED_MULTIPLIER) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * Constants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));  
        

        

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
        inOwnZone.whileTrue(new SearchForTarget(turret));


        // OPERATOR CONTROLS
        // operator.x().whileTrue(new IntakeExtend(intake));
        // operator.b().whileTrue(new IntakeRetract(intake));
        // operator.leftBumper().whileTrue(new IntakeIn(intake));
        // operator.leftBumper().whileTrue(new converyforword(index));
        // operator.leftTrigger().whileTrue(new IntakeRefund(intake));
        // operator.rightBumper().whileTrue(new leftSlingShot(shooter));
        // operator.rightBumper().whileTrue(new rightSlingShot(shooter));
        driver.x().whileTrue(new TrackHub(drivetrain, turret, hoods));

        operator.rightBumper().whileTrue(new LeftUp(index));
        operator.rightBumper().whileTrue(new RightUp(index));
        operator.rightBumper().whileTrue(new converyforword(index));
        operator.rightTrigger().whileTrue(new leftSlingShot(shooter));
        operator.rightTrigger().whileTrue(new rightSlingShot(shooter));
        operator.x().whileTrue(new SearchForTarget(turret));
        driver.rightTrigger().whileTrue(new Shoot(shooter, index));
        // operator.rightTrigger().whileTrue(new converybackwards(index));
        // operator.rightTrigger().whileTrue(new RightDown(index));
        // operator.rightTrigger().whileTrue(new LeftDown(index));
        operator.povLeft().whileTrue(new ManualRotate(turret, 12.0));
        operator.povRight().whileTrue(new ManualRotate(turret, -12.0));
        driver.povUp().whileTrue(new TESTSetHoodsHigh(hoods));
        driver.povDown().whileTrue(new TESTSetHoodsLow(hoods));

    }

    public Command getAutonomousCommand() {
        //return Commands.none();
        return autoChooser.getSelected();
    }
}
