package frc.robot.SubSystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private final NetworkTable limelight;

    public Vision(){
        limelight = NetworkTableInstance.getDefault().getTable("limelight-front");
    }

    public boolean hasTarget() {
     return limelight.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getTx() {
     return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getTa() {
     return limelight.getEntry("ta").getDouble(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight Target", hasTarget());
        SmartDashboard.putNumber("Limelight TX", getTx());
        SmartDashboard.putNumber("Limelight TA", getTa());
    }
}
