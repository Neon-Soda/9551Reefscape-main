package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LimeLightSubsystem extends SubsystemBase{

    private String limeLight;
    private CommandSwerveDrivetrain driveTrain;

    private double txoffset;
    private double tyoffset;
    private double rotyaw;
    boolean trust = false;

    public LimeLightSubsystem(CommandSwerveDrivetrain swerve, String limelightName,double txOffset, double tyOffset, double rotYaw) {
        this.driveTrain = swerve;
        this.txoffset = txOffset;
        this.tyoffset = tyOffset;
        this.rotyaw = rotYaw;
        limeLight = limelightName;
    }
    
    @Override
    public void periodic() {
        
    }
}
