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
            double tagHeight = TAG_Z[id - 1];
            
            double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_Z, tagHeight, CAMERA_PITCH, pitch);
            
            return range;
    }

    private double getDistanceY(PhotonTrackedTarget target) {
        double range = getDistanceX(target);
        double yaw = target.getYaw();

        double strafe = range * Math.tan(Math.toRadians(yaw));
        
        return strafe;
    }

    // public double[] getRelativeLocation() {
    //     PhotonPipelineResult result = aprilTagCamera.getLatestResult();
    //     if (result.hasTargets()) {
    //         PhotonTrackedTarget target = result.getBestTarget();
    //         double[] location = {getDistanceX(target), getDistanceY(target)};
    //         return location;
    //     }
    //     return null;
    // }

    public double[] getFieldLocation() {
        PhotonPipelineResult result = aprilTagCamera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            int id = target.getFiducialId();
            double xRel = getDistanceX(target);
            double yRel = getDistanceY(target);
            double xPos = xRel + TAG_X[id - 1];
            double yPos = yRel + TAG_Y[id - 1];
            double[] location = {xPos, yPos};
            return location;
        }
        return null;
    }

    @Override
    public void periodic() {
    }
}
