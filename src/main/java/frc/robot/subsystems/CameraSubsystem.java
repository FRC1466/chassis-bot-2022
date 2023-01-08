package frc.robot.subsystems;

import java.nio.file.Paths;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.CameraConstants;

public class CameraSubsystem {
    private PhotonCamera m_camera;
    private JsonNode m_node;
    
    /**
     * subsystem for the camera
     */
    public CameraSubsystem() {
        m_camera = new PhotonCamera("photonvision");

        try {
            // create object mapper instance
            ObjectMapper mapper = new ObjectMapper();
        
            m_node = mapper.readTree(Paths.get("FidID.json").toFile());
        
        } catch (Exception ex) {
            ex.printStackTrace();
        }
        
    }

    public PhotonPipelineResult getResult() {
        return m_camera.getLatestResult();
    }

    public boolean hasTargets() {
        return m_camera.getLatestResult().hasTargets();
    }

    /**
     * assumes that there are targets
     * @return id of the tag as an int
     */
    public int getBestAprilTagID() {
        return m_camera.getLatestResult().getBestTarget().getFiducialId();
    }

    /**
     * get list of all targets
     * @return list of photontrackedtargets
     */
    public List<PhotonTrackedTarget> getAllTargets() {
        return m_camera.getLatestResult().getTargets();
    }

    /**
     * 
     * @return PhotonTrackedTarget of the best target
     */
    public PhotonTrackedTarget getBestTarget() {
        return m_camera.getLatestResult().getBestTarget();
    }

    /**
     * get height of target from json file
     * @param id id of the target
     * @return height in meters as double
     */
    public double getTargetHeight(int id) {
        return m_node.get(String.valueOf(id)).get("z").asDouble();
    }

    /**
     * @return pose ambiguity
     */
    public double getPoseAmbiguity() {
        return m_camera.getLatestResult().getBestTarget().getPoseAmbiguity();
    }

    /**
     * 
     * @return a Transform3d from the camera to the best target
     */
    public Transform3d getBestTransform3d() {
        return m_camera.getLatestResult().getBestTarget().getBestCameraToTarget();
    }

    /**
     * 
     * @param id id of the target
     * @return Pose2d of the target
     */
    public Pose2d getFieldToTarget(int id) {
        return new Pose2d(
            m_node.get(String.valueOf(id)).get("x").asDouble(),
            m_node.get(String.valueOf(id)).get("y").asDouble(),
            new Rotation2d(m_node.get(String.valueOf(id)).get("r3d3").asDouble())
        );
    }

    /**
     * 
     * @return Translation2d of camera in relation to robot
     */
    public Transform2d getCameraToRobot() {
        return new Transform2d(
            new Translation2d(
                CameraConstants.CAMERA_X_METERS,
                CameraConstants.CAMERA_Y_METERS),
            new Rotation2d(
                CameraConstants.CAMERA_YAW_RAD
            )
        );
    }

    /**
     * 
     * @param id id of the target
     * @return get distance from the robot to the target in meters
     */
    public double getDistance(int id) {
        return
            PhotonUtils.calculateDistanceToTargetMeters(
                    CameraConstants.CAMERA_HEIGHT_METERS,
                    getTargetHeight(id),
                    CameraConstants.CAMERA_PITCH_RADIANS,
                    Units.degreesToRadians(m_camera.getLatestResult().getBestTarget().getPitch()));
    }
}
