package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.*;


public class Vision extends SubsystemBase {
    private final PhotonCamera aprilTagCamera = new PhotonCamera("AprilTagCamera");

    public Vision() {}

    private double getDistanceX(PhotonTrackedTarget target) {
            double pitch = target.getPitch();
            int id = target.getFiducialId();
            double tagHeight = TAG_HEIGHT[id];
            
            double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT, tagHeight, CAMERA_PITCH, pitch);
            
            return range;
    }

    private double getDistanceY(PhotonTrackedTarget target) {
        double range = getDistanceX(target);
        double yaw = target.getYaw();

        double strafe = range * Math.tan(Math.toRadians(yaw));
        
        return strafe;
    }

    public double[] getRelativeLocation() {
        PhotonPipelineResult result = aprilTagCamera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double[] location = {getDistanceX(target), getDistanceY(target)};
            return location;
        }
        return null;
    }

    @Override
    public void periodic() {
}
}
