package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.enums.FieldZoneX;
import frc.robot.enums.FieldZoneY;

public class FieldZoneManager {

    private static final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static final NetworkTable table = inst.getTable("Field Zone Manager");
    private static final DoublePublisher distToGoalPub = table.getDoubleTopic("distance to goal").publish();
    private static final StringPublisher zonePub = table.getStringTopic("Zone").publish();
    private static FieldZoneX zoneX;
    private static FieldZoneY zoneY;
    private static Alliance alliance;

    public static FieldZoneX getFieldZoneX(Translation2d translation){
        if(translation.getX() < 4){
            return FieldZoneX.BLUE;
        }
        else if(translation.getX() < 6.0){
            return FieldZoneX.BLUEBUMP;
        }
        else if(translation.getX() < 10.0){
            return FieldZoneX.NEUTRAL;
        }
        else if(translation.getX() < 12){
            return FieldZoneX.REDBUMP;
        }
        else{
            return FieldZoneX.RED;
        }
        
    }
    public static FieldZoneY getFieldZoneY(Translation2d translation){
        if(translation.getY() < 3.0){
            return FieldZoneY.SCORING_TABLE_ADJACENT;
        }
        else if(translation.getY() < 4.5){
            return FieldZoneY.CENTER;
        }
        else {
            return FieldZoneY.SCORING_TABLE_OPPOSITE;
        }
        
    }

    public static boolean inOwnZoneX(Translation2d translation){
        zoneX = getFieldZoneX(translation);
        zonePub.set(zoneX.name());
        alliance = DriverStation.getAlliance().get();
        if(alliance == Alliance.Red && (zoneX == FieldZoneX.RED || zoneX == FieldZoneX.REDBUMP)){
            return true;
        }
        else if(alliance == Alliance.Blue && (zoneX == FieldZoneX.BLUE || zoneX == FieldZoneX.BLUEBUMP)){
            return true;
        }
        return false;
    }

    public static boolean inCenterY(Translation2d translation){
        zoneY = getFieldZoneY(translation);
        if(zoneY == FieldZoneY.CENTER){
            return true;
        }
        else{
            return false;
        }
    }

    public static boolean inTrenchProtectionZone(Translation2d translation2d){
        zoneX = getFieldZoneX(translation2d);
        zoneY = getFieldZoneY(translation2d);
        if((zoneX == FieldZoneX.BLUEBUMP || zoneX == FieldZoneX.REDBUMP) && zoneY != FieldZoneY.CENTER){
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
