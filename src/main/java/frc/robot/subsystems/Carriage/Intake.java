package frc.robot.subsystems.Carriage;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Carriage;
import frc.robot.subsystems.Carriage.CarriageSystem.CarriageStates;
import frc.lib.math.Conversions;

public class Intake extends SubsystemBase{
    private TalonFX intakeMotor = new TalonFX(Carriage.intakeMotorID, Carriage.wristCanBus);
    private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    private CANrange intakeCanRange = new CANrange(Carriage.intakeCanRangeID, Carriage.wristCanBus);
    private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
    private DutyCycleOut intakeDutyCycleOut = new DutyCycleOut(0);

    public enum IntakeStates {
        Stop,
        Intake,
        Regret,
        Transport,
        Score
    }

    IntakeStates currentState = IntakeStates.Stop;
    CarriageStates carriageState = CarriageStates.OffSet;
    boolean ifAlgaeHolding = false;
    boolean ifCoralHolding = false;
    boolean ifScoring = false;
    boolean ifAlgaeTransport = false;

    public Intake() {
        configIntake();
    }

    public void configIntake(){
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40;
        intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 0.02;

        intakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        intakeConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        intakeMotor.getConfigurator().apply(intakeConfig);
        intakeCanRange.getConfigurator().apply(canRangeConfig);
    }

    public void setIntakeVol(double vol) {
        intakeDutyCycleOut.Output = vol;
        intakeMotor.setControl(intakeDutyCycleOut);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public double getIntakeVel() {
        return Conversions.RPSToMPS(intakeMotor.getVelocity().getValueAsDouble(),Carriage.intakeWheelDiameter * 2 * Math.PI);
    }

    public double getSignalStrength(boolean refresh) {
        return intakeCanRange.getSignalStrength(refresh).getValueAsDouble();
    }

    public boolean isDetected(boolean refresh) {
        return Carriage.intakeCanRangeDetectUpperLimit >= getSignalStrength(refresh) && getSignalStrength(refresh) >= Carriage.intakeCanRangeDetectLowerLimit;
    }

    public void setState(IntakeStates state) {
        currentState = state;
    }

    public void setState(IntakeStates state, CarriageStates carriageState) {
        currentState = state;
        this.carriageState = carriageState;
    }

    public void transmitCarriageState(CarriageStates state) {
        carriageState = state;
    }

    public void setAlgaeState(boolean holding) {
        ifAlgaeHolding = holding;
    }

    public boolean getAlgaeState() {
        return ifAlgaeHolding;
    }

    public void setCoralState(boolean holding) {
        ifCoralHolding = holding;
    }

    public void setIfScoring(boolean score) {
        ifScoring = score;
    }

    public void setAlgaeTransport(boolean transport) {
        ifAlgaeTransport = transport;
    }

    public void testIntakeOn() {
        setIntakeVol(Carriage.intakeVoltage);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("IntakeVelocity", getIntakeVel());
        SmartDashboard.putNumber("SignalStrength", getSignalStrength(true));
        SmartDashboard.putBoolean("IsCoralDetected", isDetected(true));
        SmartDashboard.putBoolean("IsAlgaeHolding", ifAlgaeHolding);
        SmartDashboard.putString("IntakeCarriageState", carriageState.toString());
        SmartDashboard.putString("IntakeState", currentState.toString());
        switch (currentState) {
            case Stop -> {
                if(!ifAlgaeHolding)
                    stopIntake();
                else if(ifAlgaeHolding) {
                    if(ifAlgaeTransport) {
                        setIntakeVol(Carriage.algaeTransportVoltage);
                    }
                    else {
                        setIntakeVol(Carriage.algaeHoldingVoltage);
                    }
                }
                SmartDashboard.putBoolean("IntakeOn", false);
            }

            case Intake -> {
                if(carriageState == CarriageStates.AlgaeL1 || carriageState == CarriageStates.AlgaeL2) {
                    setIntakeVol(Carriage.algaeIntakeVoltage);
                    ifAlgaeHolding = true;
                } 
                else {
                    ifAlgaeHolding = false;
                    if(isDetected(true)) {
                        stopIntake();
                    }
                    else {
                        setIntakeVol(Carriage.intakeVoltage);
                    }
                }
            }

            case Regret -> {
                if(!ifAlgaeHolding && !ifScoring) {
                    if(carriageState == CarriageStates.ReefL4)
                        setIntakeVol(Carriage.coralRegretL4Voltage);
                    else 
                        setIntakeVol(Carriage.coralRegretVoltage);
                }
            }

            case Transport -> {
                if(ifCoralHolding && !ifScoring) {
                    if(carriageState == CarriageStates.ReefL4)
                    setIntakeVol(Carriage.L4HoldingVoltage);
                }
                else if(!ifScoring) {
                    setIntakeVol(Carriage.L2L3HoldingVoltage);
                }
            }

            case Score -> {
                if(ifAlgaeHolding) {
                    ifAlgaeHolding = false;
                    setIntakeVol(Carriage.algaeScoreVoltage);
                }
                else {
                    ifCoralHolding = false;
                    setIntakeVol(Carriage.coralScoreVoltage);
                }
            }
        }
    }
}
