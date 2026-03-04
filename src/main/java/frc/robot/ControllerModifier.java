// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class ControllerModifier {
    private static SlewRateLimiter xRateLimiter = new SlewRateLimiter(5.0);
    private static SlewRateLimiter yRateLimiter = new SlewRateLimiter(5.0);

    public static double modifyX(double input){
        if(DriverStation.getAlliance().get() == Alliance.Red){
            return xRateLimiter.calculate(input);
        }
        return xRateLimiter.calculate(-input);
    }

    public static double modifyY(double input){
        if(DriverStation.getAlliance().get() == Alliance.Red){
            return yRateLimiter.calculate(input);
        }
        return yRateLimiter.calculate(-input);
    }
}
