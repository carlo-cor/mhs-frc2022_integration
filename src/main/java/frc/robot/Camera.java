package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

public class Camera {

    private UsbCamera camera;

    public Camera(int width, int height, int port){
        camera = CameraServer.startAutomaticCapture(port);
        camera.setVideoMode(PixelFormat.kMJPEG, width, height, 30);
    }
    
    public Camera(int port){
        this(160, 120, port);
    }

    public void setResolution(int width, int height){
        camera.setResolution(width, height);
    }
}