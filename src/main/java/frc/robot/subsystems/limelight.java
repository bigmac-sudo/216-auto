package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Limelight {
    private final NetworkTable limelightTable;

    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0) == 1;
    }

    public double getXOffset() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    public double getYOffset() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }
    
}
