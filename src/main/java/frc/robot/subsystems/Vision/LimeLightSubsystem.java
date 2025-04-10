package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.LimelightHelpers.RawFiducial;

public class LimeLightSubsystem extends SubsystemBase {
    private RawFiducial[] fiducials;

    public LimeLightSubsystem() {
        configLimelight();
    }

    public static class NoTargetException extends RuntimeException {
        public NoTargetException(String msg) {
            super(msg);
        }
    }

    public void configLimelight() {
        LimelightHelpers.setCameraPose_RobotSpace(VisionConstants.limeLightName, 0, 0, 0, 0, 0, 0);
        LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.limeLightName, VisionConstants.targetAprilTags);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("AprilTagTx", getTX());
        SmartDashboard.putNumber("AprilTagTy", getTY());
    }

    public RawFiducial getClosestFiducial() {
        if (fiducials == null || fiducials.length == 0) {
            throw new NoTargetException("No fiducials found");
        }

        RawFiducial closest = fiducials[0];
        double minDistance = closest.ta;

        for (RawFiducial fiducial : fiducials) {
            if (fiducial.ta > minDistance) {
                closest = fiducial;
                minDistance = fiducial.ta;
            }
        }
        return closest;
    }

    public RawFiducial getFiducialWithId(int id) {

        for (RawFiducial fiducial : fiducials) {
            if (fiducial.id == id) {
                return fiducial;
            }
        }
        throw new NoTargetException("Can't find ID: " + id);
    }

    public RawFiducial getFiducialWithId(int id, boolean verbose) {// Debug
        StringBuilder availableIds = new StringBuilder();

        for (RawFiducial fiducial : fiducials) {
            if (availableIds.length() > 0) {
                availableIds.append(", ");
            } // Error reporting
            availableIds.append(fiducial.id);

            if (fiducial.id == id) {
                return fiducial;
            }
        }
        throw new NoTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
    }

    public double getTX() {
        return LimelightHelpers.getTX(VisionConstants.limeLightName);
    }

    public double getTY() {
        return LimelightHelpers.getTY(VisionConstants.limeLightName);
    }

    public Angle getTXAngle() {
        return Angle.ofBaseUnits(LimelightHelpers.getTX(VisionConstants.limeLightName), Degrees);
    }

    public Angle getTYAngle() {
        return Angle.ofBaseUnits(LimelightHelpers.getTY(VisionConstants.limeLightName), Degrees);
    }

    public double getTA() {
        return LimelightHelpers.getTA(VisionConstants.limeLightName);
    }

    // TV stands for valid target
    public boolean hasTarget() {
        return LimelightHelpers.getTV(VisionConstants.limeLightName);
    }

    public double getClosestTX() {
        return getClosestFiducial().txnc;
    }

    public double getClosestTY() {
        return getClosestFiducial().tync;
    }

    public double getClosestTA() {
        return getClosestFiducial().ta;
    }

    public double getID_TX(int ID) {
        return getFiducialWithId(ID).txnc;
    }

    public double getID_TY(int ID) {
        return getFiducialWithId(ID).tync;
    }
}
