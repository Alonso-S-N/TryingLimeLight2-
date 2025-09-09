package frc.robot.SubSystem;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public boolean hasTarget() {
        return NetworkTableInstance.getDefault()
            .getTable("limelight").getEntry("tv").getDouble(0) == 1;
    }

    public double getTx() {
        return NetworkTableInstance.getDefault()
            .getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getTa() {
        return NetworkTableInstance.getDefault()
            .getTable("limelight").getEntry("ta").getDouble(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Limelight Target", hasTarget());
        SmartDashboard.putNumber("Limelight TX", getTx());
        SmartDashboard.putNumber("Limelight TA", getTa());
    }
}
