package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldZoneManager {
    public static boolean inOwnZone(double x){
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            if(x < 4.0){
                return true;
            }
        }
        else{
            if(x > 12.5){
                return true;
            }
        }
        return false;
    }

    public static boolean inBumpzone(double x){
        if(x > 4.0 && x < 5.5){
            return true;
        }
        else if(x > 11.0 && x < 12.5){
            return true;
        }
        else{
            return false;
        }
    }
}
