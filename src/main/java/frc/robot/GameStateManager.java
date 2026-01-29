// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.enums.GameState;

/** Add your docs here. */
public class GameStateManager {
    
    private GameState state;
    private Alliance advantageAlliance = Alliance.Red;

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("GameStateManager");
    private final StringPublisher gameStatePublisher = table.getStringTopic("Game State").publish();
    
    GameStateManager(){}

    public void setAdvantageAlliance() {
        if(DriverStation.getGameSpecificMessage() == "B"){
            advantageAlliance = Alliance.Blue;
        }
        advantageAlliance = Alliance.Red;
    }

    public GameState getState() {
        if(DriverStation.isAutonomous()){
            state = GameState.AUTO;
        }
        else if(DriverStation.getMatchTime() > 130.0){
            state = GameState.TRANSITION;
        }
        else if(DriverStation.getMatchTime() > 105.0){
            if(DriverStation.getAlliance().get() == advantageAlliance){
                state =  GameState.ACTIVE;
            }
            else{
                state =  GameState.INACTIVE;
            }
        }
        else if(DriverStation.getMatchTime() > 80.0){
            if(DriverStation.getAlliance().get() == advantageAlliance){
                state =  GameState.INACTIVE;
            }
            else{
                state =  GameState.ACTIVE;
            }
        }
        else if(DriverStation.getMatchTime() > 55.0){
            if(DriverStation.getAlliance().get() == advantageAlliance){
                state =  GameState.ACTIVE;
            }
            else{
                state =  GameState.INACTIVE;
            }
        }
        else if(DriverStation.getMatchTime() > 30.0){
            if(DriverStation.getAlliance().get() == advantageAlliance){
                state =  GameState.INACTIVE;
            }
            else{
                state =  GameState.ACTIVE;
            }
        }
        else{
            state = GameState.ENDGAME;
        }

        gameStatePublisher.set(state.toString());
        return state;
    }

}
