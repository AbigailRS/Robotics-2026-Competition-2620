package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

public class Constants {

    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final double SLOW_SPEED_MULTIPLIER = 0.3;

    public static final String CAMERA_1_NAME = "Photon Camera 1";
    public static final String CAMERA_2_NAME = "Photon Camera 2";
    public static final String CAMERA_3_NAME = "Photon Camera 3";
    public static final String CAMERA_4_NAME = "Photon Camera 4";

    public static final double CAMERA_1_TRANSLATION_X = -0.281;
    public static final double CAMERA_1_TRANSLATION_Y = 0.343;
    public static final double CAMERA_1_TRANSLATION_Z = -0.311;

    public static final double CAMERA_1_ROTATION_ROLL = 0.0;
    public static final double CAMERA_1_ROTATION_YAW = Math.toRadians(90);
    public static final double CAMERA_1_ROTATION_PITCH = Math.toRadians(0.0);

    public static final double CAMERA_2_TRANSLATION_X = -0.3429;
    public static final double CAMERA_2_TRANSLATION_Y = 0.2794;
    public static final double CAMERA_2_TRANSLATION_Z = 0.2921;

    public static final double CAMERA_2_ROTATION_ROLL = 0.0;
    public static final double CAMERA_2_ROTATION_YAW = Math.toRadians(180.0);
    public static final double CAMERA_2_ROTATION_PITCH = Math.toRadians(0.0);

    public static final double CAMERA_3_TRANSLATION_X = -0.3429;
    public static final double CAMERA_3_TRANSLATION_Y = -0.2794;
    public static final double CAMERA_3_TRANSLATION_Z = 0.2921;

    public static final double CAMERA_3_ROTATION_ROLL = 0.0;
    public static final double CAMERA_3_ROTATION_YAW = Math.toRadians(180.0);
    public static final double CAMERA_3_ROTATION_PITCH = Math.toRadians(0.0);

    public static final double CAMERA_4_TRANSLATION_X = -0.281;
    public static final double CAMERA_4_TRANSLATION_Y = -0.343;
    public static final double CAMERA_4_TRANSLATION_Z = 0.311;

    public static final double CAMERA_4_ROTATION_ROLL = 0.0;
    public static final double CAMERA_4_ROTATION_YAW = Math.toRadians(270.0);
    public static final double CAMERA_4_ROTATION_PITCH = Math.toRadians(0.0);

    public static final Translation2d POSE_RED_HUB = new Translation2d(12.0, 4.0);
    public static final Translation2d POSE_BLUE_HUB = new Translation2d(4.6, 4.0);

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final double LATENCY_CONSTANT = 0.15;
    public static final double DRIVE_SHOOT_MODIFIER = 0.25;
    public static final double DRIVE_TURN_MODIFIER = 0.65;
    public static final double DRIVE_MAX_SPEED_MODIFIER = 0.8;


    //Turret Constants
    public static final String PRIMARY_LL_NAME = "limelight-left";
    public static final String SECONDARY_LL_NAME = "limelight-right";
    public static final double TURRET_P = 0.1;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.0;
    public static final double TURRET_MAX_OUTPUT_VOLTS = 3.0;
    public static final double TURRET_RAMPRATE = 0.0;   //Defined as amount of time to go from 0 to 12v
    public static final InvertedValue TURRET_INVERSION = InvertedValue.Clockwise_Positive;
    public static final double TURRET_CURRENT_LIMIT = 18; //amps
    public static final NeutralModeValue TURRET_NEUTRALMODE = NeutralModeValue.Coast;
    public static final double TURRET_SEARCH_VOLTAGE = 6.0;
    public static final double TURRET_STALL_VELOCITY = 0.2;
    public static final double TURRET_STALL_CURRENT = 15.0;

    public static final int TURRET_CANCODER_ID = 35;
    public static final double TURRET_LEFT_LIMIT = -100.0;
    public static final double TURRET_RIGHT_LIMIT = 100.0; 
    public static final double TURRET_CANCODER_OFFSET = 0.0;
    public static final double TURRET_MOTOR_TO_TURRET_RATIO = 35.15;
    public static final double TURRET_LEFT_ENDSTOP_OFFSET = 17.25;

    //Climb Constants
    public static final int CLIMB_LEFT_CANID = 40;
    public static final int CLIMB_RIGHT_CANID = 41;

    public static final double LEFT_CLIMB_MAX_VOLTAGE = 12;
    public static final double LEFT_CLIMB_VOLTAGE_PERCENTAGE = 0.75;
    public static final double RETRACT_CLIMB_MAX_VOLTAGE = -12;
    public static final double RETRACT_CLIMB_VOLTAGE_PERCENTAGE = -0.75; 

    //Shooter Constants
    public static final int TURRET_CANID = 20;
    public static final int LEFT_SHOOT_CANID = 21;
    public static final int RIGHT_SHOOT_CANID = 22;

    public static final int LEFT_LASERCAN_CANID = 23;
    public static final int RIGHT_LASERCAN_CANID = 24;
    
    public static final double LEFT_SLING_MAX_VOLTAGE = 12;
    public static final double VOLTAGE_PERCENTAGE_LEFT = 0.95;
    public static final double RIGHT_SLING_MAX_VOLTAGE = 12;
    public static final double VOLTAGE_PERCENTAGE_RIGHT = 0.95;

    public static final double VELOCITY_LEFT_SLING = 65.0;
    public static final double VELOCITY_RIGHT_SLING = 65.0;

    public static final double LAZY_SUSAN_STOP = 0.0;
    public static final double LEFT_SLING_STOP = 0.0;
    public static final double RIGHT_SLING_STOP = 0.0;

    public static final int LASERCAN_LEFT_THRESHOLD_MM = 5;
    public static final int LASERCAN_RIGHT_THRESHOLD_MM = 5;

    public static final double SHOOTER_LEFT_ALLOWABLE_ERROR = 2.0;
    public static final double SHOOTER_RIGHT_ALLOWABLE_ERROR = 2.0;

    public static final double SHOOTER_PASS_VELOCITY = 90.0;
    public static final double SHOOTER_MANUAL_SHOOT_PERCENTAGE = 0.25;

    public static final double SHOOT_V2_CONSTANT_SPEED = 70.0;
    
    //Intake Constants
    public static final int LOWER_WHEEL_INTAKE_CANID = 30;
    public static final int UPPER_WHEEL_INTAKE_CANID = 31;

    public static final double MAX_INTAKE_VOLTAGE = 12;
    public static final double INTAKE_VOLTAGE_PERCENTAGE = 0.45;
    public static final double MAX_EXTEND_VOLTAGE = 12;
    public static final double EXTEND_VOLTAGE_PERCENTAGE = 0.5;
    public static final double EXTEND_POSITION_OUT = 20.0;
    public static final double EXTEND_POSITION_IN = 0.0;
    public static final double EXTEND_POSITION_OSCILLATE = 40.0;

    public static final double INTAKE_POSITION_IN = 0.0;
    public static final double INTAKE_POSITION_OUT = 91.0;

    public static final double INTAKE_STOP = 0.0;

    public static final double INTAKE_IN_POSITION_ERROR = 5.0;
    public static final double INTAKE_EXTENSION_CURRENT_LIMIT = 18.0;
    public static final double INTAKE_CURRENT_THRESHOLD = 18.0;
    public static final double INTAKE_EXTEND_ZERO_PERCENTAGE = 0.3;

    public static final double INTAKE_EXTEND_RAMP_RATE = 0.2;
    public static final double INTAKE_ROTATE_RAMP_RATE = 0.2;

    //Indexer
    public static final int CONVEYOR_CANID = 50;
    public static final int RIGHT_INDEXER_CANID = 52;
    public static final int LEFT_INDEXER_CANID = 51;

    public static final double CONVEYOR_VOLTAGE = 4;
    public static final double RIGHT_ROCK_SMUSHER_VOLTAGE = 1.5;
    public static final double LEFT_ROCK_SMUSHER_VOLTAGE = 1.5;
    public static final double MAX_CONVEYOR_VOLTAGE = 12;
    public static final double CONVEYOR_VOLTAGE_PERCENTAGE = 0.7;
    public static final double MAX_LEFT_SMUSHER_VOLTAGE = 12;
    public static final double LEFT_ROCK_SMUSHER_VOLTAGE_PERCENTAGE = 0.5;
    public static final double MAX_RIGHT_SMUSHER_VOLTAGE = 12;
    public static final double RIGHT_ROCK_SMUSHER_VOLTAGE_PERCENTAGE = 0.5;

    public static final InvertedValue INDEX_LEFT_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue INDEX_RIGHT_INVERT = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue INDEX_CONVERY_INVERT  = InvertedValue.CounterClockwise_Positive;

    public static final double LEFT_INDEXER_STOP = 0.0;
    public static final double RIGHT_INDEXER_STOP = 0.0;
    public static final double CONVEYOR_FORWARDS_STOP = 0.0;
    public static final double CONVEYOR_BACKWARDS_STOP = 0.0;

    public static final double INDEXER_LEFT_CURRENT_LIMIT = 20.0;
    public static final double INDEXER_RIGHT_CURRENT_LIMIT = 20.0;
    public static final double INDEXER_CONVEYOR_CURRENT_LIMIT = 40.0;
    

    //Hoods
    public static final int LEFT_HOOD_SERVO = 0;
    public static final int RIGHT_HOOD_SERVO = 1;
    public static final double HOOD_LEFT_HIGH_POSITION = .01;
    public static final double HOOD_LEFT_LOW_POSITION = 0.99;
    public static final double HOOD_RIGHT_HIGH_POSITION = 0.01;
    public static final double HOOD_RIGHT_LOW_POSITION = .99;

}