package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.camera.PhotonHelper;
import frc.robot.lib.camera.PhotonHelper.MeasurementProvider;
import frc.robot.lib.subsystems.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonHelper leftCamera;
    private final MeasurementProvider measurementProvider;
    private final StructPublisher<Pose2d> predictPose = NetworkTableInstance.getDefault()
        .getStructTopic("AdvantageScope/PredictPose", Pose2d.struct).publish();
    // private final PhotonHelper rightCamera;
    
    public VisionSubsystem(MeasurementProvider measurementProvider) {
        super("Vision", false);
        this.leftCamera = new PhotonHelper(
            "left-Cam",
            new Translation3d(0.0, 0.0, 0.068612),
            Units.degreesToRadians(-10.0), Units.degreesToRadians(0.0));
        this.measurementProvider = measurementProvider;
        // this.rightCamera = new PhotonHelper(
        //     "right-Cam",
        //     new Translation3d(0.261372, -0.236374, 0.224837),
        //     new Rotation3d(0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(0.0)),
        //     estimateConsumer);
    }

    @Override
    public void periodic() {
        this.predictPose.accept(this.leftCamera.getRobotField());
        if (this.leftCamera.hasTarget())  this.measurementProvider.addVisionMeasurement(this.leftCamera.getRobotField(), this.leftCamera.getTimeStamp());
       
    }

    @Override
    public void putDashboard() {
        SmartDashboard.putString("AprilTagPose",this.leftCamera.getRobotField().toString());
        SmartDashboard.putNumber("TimeStamp", this.leftCamera.getTimeStamp());
    }
}
