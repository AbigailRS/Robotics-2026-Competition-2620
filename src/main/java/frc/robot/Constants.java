package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static final int TURRET_ROTATE_ID = 21;

    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    public static final double CAMERA_LEFT_TRANSLATION_X = 0.3429;
    public static final double CAMERA_LEFT_TRANSLATION_Y = 0.1143;
    public static final double CAMERA_LEFT_TRANSLATION_Z = 0.0;

    public static final double CAMERA_LEFT_ROTATION_ROLL = 0.0;
    public static final double CAMERA_LEFT_ROTATION_YAW = 0.0;
    public static final double CAMERA_LEFT_ROTATION_PITCH = Math.toRadians(30.0);

    public static final double CAMERA_RIGHT_TRANSLATION_X = 0.3429;
    public static final double CAMERA_RIGHT_TRANSLATION_Y = -0.1143;
    public static final double CAMERA_RIGHT_TRANSLATION_Z = 0.0;

    public static final double CAMERA_RIGHT_ROTATION_ROLL = 0.0;
    public static final double CAMERA_RIGHT_ROTATION_YAW = 0.0;
    public static final double CAMERA_RIGHT_ROTATION_PITCH = Math.toRadians(30.0);

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    

    //Climb Constants
    public static final int CLIMB_LEFT_CANID = 40;
    public static final int CLIMB_RIGHT_CANID = 41;

    public static final double LEFT_CLIMB_MAX_VOLTAGE = 12;
    public static final double LEFT_CLIMB_VOLTAGE_PERCENTAGE = 0.75;
    public static final double RIGHT_CLIMB_MAX_VOLTAGE = 12;
    public static final double RIGHT_CLIMB_VOLTAGE_PERCENTAGE = 0.75; 

    //Shooter Constants
    public static final int LAZY_SHOOT_CANID = 20;
    public static final int LEFT_SHOOT_CANID = 21;
    public static final int RIGHT_SHOOT_CANID = 22;
    
    public static final double MAX_VOLTAGE = 12;
    public static final double VOLTAGE_PERCENTAGE = 0.75;
    public static final double LEFT_SLING_MAX_VOLTAGE = 12;
    public static final double VOLTAGE_PERCENTAGE_LEFT = 0.75;
    public static final double RIGHT_SLING_MAX_VOLTAGE = 12;
    public static final double VOLTAGE_PERCENTAGE_RIGHT = 0.75;

    public static final double LAZY_SUSAN_STOP = 0.0;
    public static final double LEFT_SLING_STOP = 0.0;
    public static final double RIGHT_SLING_STOP = 0.0;
    
    //Intake Constants
    public static final int LOWER_WHEEL_INTAKE_CANID = 30;
    public static final int UPPER_WHEEL_INTAKE_CANID = 31;

    public static final double MAX_INTAKE_VOLTAGE = 12;
    public static final double INTAKE_VOLTAGE_PERCENTAGE = 0.5;
    public static final double MAX_EXTEND_VOLTAGE = 12;
    public static final double EXTEND_VOLTAGE_PERCENTAGE = 0.5;

    public static final double INTAKE_STOP = 0;



    //Indxer
    public static final int CONVEYOR_CANID = 50;
    public static final int RIGHT_INDEXER_CANID = 51;
    public static final int LEFT_INDEXER_CANID = 52;

    public static final int CONVEYOR_VOLTAGE = 4;
    public static final int RIGHT_ROCK_SMUSHER_VOLTAGE = 4;
    public static final int LEFT_ROCK_SMUSHER_VOLTAGE = 4;
    public static final double MAX_CONVEYOR_VOLTAGE = 12;
    public static final double CONVEYOR_VOLTAGE_PERCENTAGE = 0.5;
    public static final double MAX_LEFT_SMUSHER_VOLTAGE = 12;
    public static final double LEFT_ROCK_SMUSHER_VOLTAGE_PERCENTAGE = 0.5;
    public static final double MAX_RIGHT_SMUSHER_VOLTAGE = 12;
    public static final double RIGHT_ROCK_SMUSHER_VOLTAGE_PERCENTAGE = 0.5;

    public static final double LEFT_INDEXER_STOP = 0.0;
    public static final double RIGHT_INDEXER_STOP = 0.0;
    public static final double CONVEYOR_FORWARDS_STOP = 0.0;
    public static final double CONVEYOR_BACKWARDS_STOP = 0.0;


}


/* package frc.robot;

public class Constants {

    //Climb Constants
    public static final int CLIMB_LEFT_CANID = 40;
    public static final int CLIMB_RIGHT_CANID = 41;

    public static final double leftClimbMaxVoltage = 12;
    public static final double leftClimbVoltagePercentage = 0.75;
    public static final double rightClimbMaxVoltage = 12;
    public static final double rightClimbVoltagePercentage = 0.75; 

    //Shooter Constants
    public static final int LAZY_SHOOT_CANID = 20;
    public static final int LEFT_SHOOT_CANID = 21;
    public static final int RIGHT_SHOOT_CANID = 22;
    
    public static final double MAX_VOLTAGE = 12;
    public static final double VOLTAGE_PERCENTAGE = 0.75;
    public static final double LEFT_SLING_MAX_VOLTAGE = 12;
    public static final double VOLTAGE_PERCENTAGE_LEFT = 0.75;
    public static final double RIGHT_SLING_MAX_VOLTAGE = 12;
    public static final double VOLTAGE_PERCENTAGE_RIGHT = 0.75;

    public static final double LAZY_SUSAN_STOP = 0.0;
    public static final double LEFT_SLING_STOP = 0.0;
    public static final double RIGHT_SLING_STOP = 0.0;
    
    //Intake Constants
    public static final int LOWER_WHEEL_INTAKE_CANID = 30;
    public static final int UPPER_WHEEL_INTAKE_CANID = 31;

    public static final double MAX_INTAKE_VOLTAGE = 12;
    public static final double INTAKE_VOLTAGE_PERCENTAGE = 0.5;
    public static final double MAX_EXTEND_VOLTAGE = 12;
    public static final double EXTEND_VOLTAGE_PERCENTAGE = 0.5;

    public static final double INTAKE_STOP = 0.0;



    //Indxer
    public static final int CONVEYOR_CANID = 50;
    public static final int RIGHT_INDEXER_CANID = 51;
    public static final int LEFT_INDEXER_CANID = 52;

    public static final int converyVoltage = 4;
    public static final int rightRockSmusherVoltage = 4;
    public static final int leftRockSmusherVoltage = 4;
    public static final double MAX_CONVEYOR_VOLTAGE = 12;
    public static final double CONVEYOR_VOLTAGE_PERCENTAGE = 0.5;
    public static final double MAX_LEFT_SMUSHER_VOLTAGE = 12;
    public static final double LEFT_ROCK_SMUSHER_VOLTAGE_PERCENTAGE = 0.5;
    public static final double MAX_RIGHT_SMUSHER_VOLTAGE = 12;
    public static final double RIGHT_ROCK_SMUSHER_VOLTAGE_PERCENTAGE = 0.5;

    public static final double LEFT_INDEXER_STOP = 0.0;
    public static final double RIGHT_INDEXER_STOP = 0.0;
    public static final double CONVEYOR_FORWARDS_STOP = 0.0;
    public static final double CONVEYOR_BACKWARDS_STOP = 0.0;


}
*/