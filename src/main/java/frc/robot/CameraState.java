package frc.robot;

public class CameraState {
    private static boolean cameraEnabled;

    CameraState(){
        cameraEnabled = true;
    }

    public void setCameraState(boolean state){
        cameraEnabled = state;
    }

    public boolean getCameraState(){
        return cameraEnabled;
    }
}
