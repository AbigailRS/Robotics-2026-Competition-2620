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
}
