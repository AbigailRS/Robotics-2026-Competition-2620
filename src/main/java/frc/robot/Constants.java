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
    public static final int climbLeft = 40;
    public static final int climbRight = 41;

    public static final double leftClimbMaxVoltage = 12;
    public static final double leftClimbVoltagePercentage = 0.75;
    public static final double rightClimbMaxVoltage = 12;
    public static final double rightClimbVoltagePercentage = 0.75; 

    //Shooter Constants
    public static final int lazyShoot = 20;
    public static final int leftShoot = 21;
    public static final int rightShoot = 22;
    
    public static final double maxVoltage = 12;
    public static final double voltagePercentage = 0.75;
    public static final double leftSlingMaxVoltage = 12;
    public static final double voltagePercentageLeft = 0.75;
    public static final double rightSlingMaxVoltage = 12;
    public static final double voltagePercentageRight = 0.75;

    public static final double lazySusanStop = 0.0;
    public static final double leftSlingShotStop = 0.0;
    public static final double rightSlingShotStop = 0.0;
    
    //Intake Constants
    public static final int lowerWheelIntake = 30;
    public static final int upperWheelIntake = 31;

    public static final double maxIntakeVoltage = 12;
    public static final double intakeVoltagePercentage = 0.5;
    public static final double maxExtendVoltage = 12;
    public static final double extendVoltagePercentage = 0.5;



    //Indxer
    public static final int converorInxder = 50;
    public static final int rightInxder = 51;
    public static final int leftInxder = 52;

    public static final int converyVoltage = 4;
    public static final int rightRockSmusherVoltage = 4;
    public static final int leftRockSmusherVoltage = 4;
    public static final double maxConveryVoltage = 12;
    public static final double converyVoltagePercentage = 0.5;
    public static final double maxLeftRockSmusherVoltage = 12;
    public static final double leftRockSmusherVoltagePercentage = 0.5;
    public static final double maxRightRockSmusherVoltage = 12;
    public static final double rightRockSmusherVoltagePercentage = 0.5;

    public static final double LeftInxderStop = 0.0;
    public static final double RightInxderStop = 0.0;
    public static final double ConveryForwardStop = 0.0;
    public static final double ConveryBackwardStop = 0.0;


}
