package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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

    public static double getDistanceTogoal(Translation2d robotTranslation){
        if(DriverStation.getAlliance().get() == Alliance.Red){
            return Math.sqrt(Math.pow(Constants.POSE_RED_HUB.getX() - robotTranslation.getX(), 2) + Math.pow(Constants.POSE_RED_HUB.getY() - robotTranslation.getY(), 2));
        }
        else{
            return Math.sqrt(Math.pow(Constants.POSE_BLUE_HUB.getX() - robotTranslation.getX(), 2) + Math.pow(Constants.POSE_BLUE_HUB.getY() - robotTranslation.getY(), 2));
        }
    }
}
