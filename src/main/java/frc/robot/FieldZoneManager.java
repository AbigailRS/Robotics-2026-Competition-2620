package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.enums.FieldZone;

public class FieldZoneManager {

    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable table = inst.getTable("Field Zone Manager");
    private static final DoublePublisher distToGoalPub = table.getDoubleTopic("distance to goal").publish();
    private static final StringPublisher zonePub = table.getStringTopic("Zone").publish();
    private static FieldZone zone;
    private static Alliance alliance;

    public static FieldZone getFieldZone(Translation2d translation){
        if(translation.getX() < 4.0){
            return FieldZone.BLUE;
        }
        else if(translation.getX() < 6.0){
            return FieldZone.BLUEBUMP;
        }
        else if(translation.getX() < 10.0){
            return FieldZone.NEUTRAL;
        }
        else if(translation.getX() < 12.0){
            return FieldZone.REDBUMP;
        }
        else{
            return FieldZone.RED;
        }
        
    }

    public static boolean inOwnZone(Translation2d translation){
        zone = getFieldZone(translation);
        zonePub.set(zone.name());
        alliance = DriverStation.getAlliance().get();
        if(alliance == Alliance.Red && zone == FieldZone.RED){
            return true;
        }
        else if(alliance == Alliance.Blue && zone == FieldZone.BLUE){
            return true;
        }
        return false;
    }

    public static double getDistanceTogoal(Translation2d robotTranslation){
        double dist = 0;
        if(DriverStation.getAlliance().get() == Alliance.Red){
            dist =  Math.sqrt(Math.pow(Constants.POSE_RED_HUB.getX() - robotTranslation.getX(), 2) + Math.pow(Constants.POSE_RED_HUB.getY() - robotTranslation.getY(), 2));
        }
        else{
            dist =  Math.sqrt(Math.pow(Constants.POSE_BLUE_HUB.getX() - robotTranslation.getX(), 2) + Math.pow(Constants.POSE_BLUE_HUB.getY() - robotTranslation.getY(), 2));
        }
        distToGoalPub.set(Math.abs(dist));
        return Math.abs(dist);
    }
}
