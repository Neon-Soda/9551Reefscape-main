package frc.robot.subsystems.Carriage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Carriage;

public class Wrist extends SubsystemBase{
    private TalonFX wristMotor = new TalonFX(Carriage.wristMotorID, Carriage.wristCanBus);
    private TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    private MotionMagicVoltage wristMagic = new MotionMagicVoltage(0);

    double currentRotation = 0.0;

    public Wrist() {
        configWrist();
    }

    public void configWrist() {
        wristConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        wristConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        wristConfig.CurrentLimits.SupplyCurrentLimit = 40;
        wristConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
        wristConfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;

        wristConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        wristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        wristConfig.Slot0.kP = Carriage.wristKP;
        wristConfig.Slot0.kI = Carriage.wristKI;
        wristConfig.Slot0.kD = Carriage.wristKD;
        wristConfig.Slot0.kV = Carriage.wristKV;
        wristConfig.Slot0.kS = Carriage.wristKS;
        wristConfig.Slot0.kA = Carriage.wristKA;
        wristConfig.Slot0.kG = Carriage.wristKG;
        wristConfig.MotionMagic.MotionMagicAcceleration = Carriage.wristAcc;
        wristConfig.MotionMagic.MotionMagicCruiseVelocity = Carriage.wristVel;
        wristConfig.MotionMagic.MotionMagicJerk = Carriage.wristJer;

        wristConfig.Feedback.SensorToMechanismRatio = Carriage.wristMotorRatio;

        wristMotor.getConfigurator().apply(wristConfig);
        resetPosistion();
    }

    private void resetPosistion() {
        wristMotor.getConfigurator().setPosition(0);
    }

    public void setWristRotation(double rotation) {
        rotation = MathUtil.clamp(rotation, Carriage.minRotation,Carriage.maxRotation);
        currentRotation = rotation;
        wristMagic.Position = rotation;
        wristMotor.setControl(wristMagic);
    }
    
    public void setWristDutyCycleOut(double outputpercentage) {
        DutyCycleOut voltageout = new DutyCycleOut(outputpercentage);
        wristMotor.setControl(voltageout);
    }

    public void stopWrist() {
        wristMotor.stopMotor();
    }

    public double getRotation() {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public boolean atDesiredRotation(double rotation) {
        return Math.abs(getRotation() - rotation) < Carriage.wristTolerance;
    }

    @Override
    public void periodic() {;
        SmartDashboard.putNumber("WristRotation", getRotation()); 
        SmartDashboard.putNumber("WristTarget", currentRotation);
        // SmartDashboard.putNumber("WristAfterTolorance", Math.abs(getRotation() - currentRotation));
        SmartDashboard.putBoolean("WristAtPosition", atDesiredRotation(currentRotation));
    }
}
