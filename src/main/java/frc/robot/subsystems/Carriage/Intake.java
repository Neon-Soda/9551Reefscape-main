package frc.robot.subsystems.Carriage;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Carriage;
import frc.robot.subsystems.Carriage.CarriageSystem.CarriageStates;

public class Intake extends SubsystemBase{
    private TalonFX intakeMotor = new TalonFX(Carriage.intakeMotorID, Carriage.wristCanBus);
    private TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    private CANrange intakeCanRange = new CANrange(Carriage.intakeCanRangeID, Carriage.wristCanBus);
    private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
    private DutyCycleOut intakeDutyCycleOut = new DutyCycleOut(0);

    // Intake has too many states and will be complex to do in carriageSystem.java, so seperate it into its own states
    public enum IntakeStates {
        Stop,
        Intake,
        Regret,
        Transport,
        Score
    }

    IntakeStates currentState = IntakeStates.Stop;
    CarriageStates carriageState = CarriageStates.OffSet;

    // Flags for different states
    boolean ifAlgaeHolding = false;
    boolean ifCoralHolding = false;
    boolean ifScoring = false;
    boolean ifAlgaeTransport = false;
    boolean ifRegreting = false;

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

    // Return the reading from canrange
    public double getSignalStrength(boolean refresh) {
        return intakeCanRange.getSignalStrength(refresh).getValueAsDouble();
    }

    // The embeded isDetected() method will always be true since the other side of the intake will always be detected
    // Checking the distance value return by canrange meet the [coral in] value or not
    public boolean isDetected(boolean refresh) {
        return Carriage.intakeCanRangeDetectUpperLimit >= getSignalStrength(refresh) && getSignalStrength(refresh) >= Carriage.intakeCanRangeDetectLowerLimit;
    }

    public void setState(IntakeStates state) {
        currentState = state;
    }

    // Overrides setState() method for situations that requires the state of elevator
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

    public void setIfRegret(Boolean regret) {
        ifRegreting = regret;
    }

    public void testIntakeOn() {
        setIntakeVol(Carriage.intakeVoltage);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("IntakeVelocity", getIntakeVel());
        SmartDashboard.putNumber("SignalStrength", getSignalStrength(true));
        SmartDashboard.putBoolean("IsCoralDetected", isDetected(true));
        // SmartDashboard.putBoolean("IsAlgaeHolding", ifAlgaeHolding);
        // SmartDashboard.putString("IntakeCarriageState", carriageState.toString());
        // SmartDashboard.putString("IntakeState", currentState.toString());

        switch (currentState) {
            case Stop -> {
                // Needs to lock algae in place and more voltage to lock algae when the wrist is flipping
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
            }

            case Intake -> {
                // Intake algae
                if(carriageState == CarriageStates.AlgaeL1 || carriageState == CarriageStates.AlgaeL2) {
                    setIntakeVol(Carriage.algaeIntakeVoltage);
                    ifAlgaeHolding = true;
                } 
                // Intake coral until detected by canrange
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
                // Move coral back since sometimes canrange overshoots and L4's gavity
                if(!ifAlgaeHolding && !ifScoring) {
                    ifRegreting = true;
                    if(carriageState == CarriageStates.ReefL4)
                        setIntakeVol(Carriage.coralRegretL4Voltage);
                    else 
                        setIntakeVol(Carriage.coralRegretVoltage);
                }
            }

            // Lock coral in place while elevator moving
            case Transport -> {
                if(ifCoralHolding && !ifScoring && !ifRegreting) {
                    if(carriageState == CarriageStates.ReefL4)
                    setIntakeVol(Carriage.L4HoldingVoltage);
                }
                else if(!ifScoring && !ifRegreting) {
                    setIntakeVol(Carriage.L2L3HoldingVoltage);
                }
            }

            case Score -> {
                // Algae score
                if(ifAlgaeHolding) {
                    setIntakeVol(Carriage.algaeScoreVoltage);
                    ifAlgaeHolding = false;
                }
                // Coral socre
                else {
                    setIntakeVol(Carriage.coralScoreVoltage);
                    ifCoralHolding = false;
                }
            }
        }
    }
}
