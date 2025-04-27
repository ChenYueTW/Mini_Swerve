package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.camera.PhotonHelper;
import frc.robot.lib.camera.PhotonHelper.EstimateConsumer;
import frc.robot.lib.subsystems.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonHelper leftCamera;
    // private final PhotonHelper rightCamera;
    
    public VisionSubsystem(EstimateConsumer estimateConsumer) {
        super("Vision", false);
        this.leftCamera = new PhotonHelper(
            "left-Cam",
            new Translation3d(0.0, 0.0, 0.068612),
            new Rotation3d(0.0, Units.degreesToRadians(10.0), Units.degreesToRadians(-15.0)),
            estimateConsumer);
        // this.rightCamera = new PhotonHelper(
        //     "right-Cam",
        //     new Translation3d(0.261372, -0.236374, 0.224837),
        //     new Rotation3d(0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(0.0)),
        //     estimateConsumer);
    }

    @Override
    public void periodic() {
        this.leftCamera.periodic();
        // this.rightCamera.periodic();
    }

    @Override
    public void putDashboard() {
        
    }
}