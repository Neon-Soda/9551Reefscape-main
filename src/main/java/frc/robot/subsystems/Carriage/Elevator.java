package frc.robot.subsystems.Carriage;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Carriage;

public class Elevator extends SubsystemBase{
    private TalonFX leftMotor = new TalonFX(Carriage.leftElevatorMotorID, Carriage.carriageCanbus);
    private TalonFX rightMotor = new TalonFX(Carriage.rightElevatorMotorID, Carriage.carriageCanbus);
    private TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration();
    private TalonFXConfiguration rightMotorConfig = new TalonFXConfiguration();
    private MotionMagicVoltage elevatorMagic = new MotionMagicVoltage(0);

    double currentPosition = 0.0;


    public Elevator() {
        configElevator();
    }

    public void configElevator() {
        leftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        rightMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        leftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        leftMotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = 60;
        leftMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.01;

        rightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        rightMotorConfig.CurrentLimits.SupplyCurrentLowerLimit  = 60;
        rightMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.01;

        leftMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
        leftMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
        leftMotorConfig.Feedback.SensorToMechanismRatio = Carriage.elevatorMotorRatio;

        rightMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
        rightMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
        rightMotorConfig.Feedback.SensorToMechanismRatio = Carriage.elevatorMotorRatio;

        leftMotorConfig.Slot0.kP = Carriage.elevatorKP;
        leftMotorConfig.Slot0.kI = Carriage.elevatorKI;
        leftMotorConfig.Slot0.kD = Carriage.elevatorKD;
        leftMotorConfig.Slot0.kV = Carriage.elevatorKV;
        leftMotorConfig.Slot0.kS = Carriage.elevatorKS;
        leftMotorConfig.Slot0.kA = Carriage.elevatorKA;
        leftMotorConfig.Slot0.kG = Carriage.elevatorKG;
        leftMotorConfig.MotionMagic.MotionMagicAcceleration = Carriage.Acc;
        leftMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Carriage.Vel;
        leftMotorConfig.MotionMagic.MotionMagicJerk = Carriage.Jer;

        rightMotorConfig.Slot0.kP = Carriage.elevatorKP;
        rightMotorConfig.Slot0.kI = Carriage.elevatorKI;
        rightMotorConfig.Slot0.kD = Carriage.elevatorKD;
        rightMotorConfig.Slot0.kV = Carriage.elevatorKV;
        rightMotorConfig.Slot0.kS = Carriage.elevatorKS;
        rightMotorConfig.Slot0.kA = Carriage.elevatorKA;
        rightMotorConfig.Slot0.kG = Carriage.elevatorKG;
        rightMotorConfig.MotionMagic.MotionMagicAcceleration = Carriage.Acc;
        rightMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Carriage.Vel;
        rightMotorConfig.MotionMagic.MotionMagicJerk = Carriage.Jer;

        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);
        resetPosistion();
    }

    public void resetPosistion() {
        leftMotor.getConfigurator().setPosition(0);
        rightMotor.getConfigurator().setPosition(0);
    }

    public void setElevatorHeight(double position) {
        position = MathUtil.clamp(position, Carriage.minHeight,Carriage.maxHeight);
        elevatorMagic.Position = position;
        currentPosition = position;
        leftMotor.setControl(elevatorMagic);
        rightMotor.setControl(elevatorMagic);
    }

    public void setElevatorDutyCycleOut(double outputpercentage) {
        DutyCycleOut voltageout = new DutyCycleOut(outputpercentage);
        leftMotor.setControl(voltageout);
        rightMotor.setControl(voltageout);
    }

    public void stopElevator() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public void climbUp() {
        setElevatorDutyCycleOut(Carriage.elevatorClimbUpDutyCycleOut);
    }

    public void climbDown() {
        setElevatorDutyCycleOut(Carriage.elevatorClimbDownDutyCycleOut);
    }

    public double getHeight(){
        return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2;
    }

    public boolean atDesiredHeight(double height){
        return Math.abs(getHeight() - height) < Carriage.elevatorTolerance;
    }

    public boolean atDesiredHeight(double height, boolean topper) {
        if(topper) {
            return Math.abs(getHeight() - height) < Carriage.elevatorTopperTolerance;
        }
        return atDesiredHeight(height);
    }


    @Override
    public void periodic() {
         SmartDashboard.putNumber("ElevatorHeight",getHeight());
         SmartDashboard.putNumber("ElevatorCurrentTarget", currentPosition);
         SmartDashboard.putBoolean("ElevatorAtDesiredHeight", atDesiredHeight(currentPosition));
        //  SmartDashboard.putNumber("ElevatorLeft", leftMotor.getPosition().getValueAsDouble());
        //  SmartDashboard.putNumber("ElevatorRight", rightMotor.getPosition().getValueAsDouble());
    }
}
