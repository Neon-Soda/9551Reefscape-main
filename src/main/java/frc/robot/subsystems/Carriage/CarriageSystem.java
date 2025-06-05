package frc.robot.subsystems.Carriage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Carriage;
import frc.robot.subsystems.Carriage.Intake.IntakeStates;

// This is the controller of the entire carriage including elevator, intake, and wrist
// Different cases of the robot has been divided into different states
public class CarriageSystem extends SubsystemBase{
    private Elevator elevator;
    private Intake intake;
    private Wrist wrist;

    // States of the carriage
    public enum CarriageStates {
        OffSet,
        ReefL2,
        ReefL3,
        ReefL4,
        AlgaeL1,
        AlgaeL2,
        Net,
        Processor,
        Climb
    }

    CarriageStates currentState = CarriageStates.OffSet;

    boolean forceWristRotate = false;

    public CarriageSystem(Elevator elevator, Intake intake, Wrist wrist) {
        this.elevator = elevator;
        this.intake = intake;
        this.wrist = wrist;
    }

    public void setState(CarriageStates state) {
        currentState = state;
    }

    public CarriageStates getState() {
        return currentState;
    }

    public void setForceWristRotate(boolean force) {
        forceWristRotate = force;
    }

    public boolean getForceWristRotate() {
        return forceWristRotate;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putString("CarriageState", currentState.toString());

        switch (currentState) {
            case OffSet -> {
                // Algae offset
                if(intake.getAlgaeState()) {
                    // The elevatorOffSetPositon is negative due to the error accumulated in elevator
                    if(elevator.atDesiredHeight(Math.abs(Carriage.elevatorOffSetPosition + Carriage.elevatorOffSetTolerace)) || forceWristRotate) {
                        wrist.setWristRotation(Carriage.wristAlgaeOffSetPosition);
                    }
                    else if(!elevator.atDesiredHeight(Math.abs(Carriage.elevatorOffSetPosition + Carriage.elevatorOffSetTolerace))){
                        wrist.setWristRotation(Carriage.wristAlgaeLiftPosition);
                        if(wrist.atDesiredRotation(Carriage.wristAlgaeLiftPosition)) {
                            elevator.setElevatorHeight(Carriage.elevatorAlgaeOffSetPosition);
                            if(elevator.atDesiredHeight(Carriage.elevatorAlgaeOffSetPosition) || forceWristRotate) {
                                wrist.setWristRotation(Carriage.wristAlgaeOffSetPosition);
                            }
                        }
                    }
                }
                // Coral offset
                else if(intake.getAlgaeState() == false) {
                    if(elevator.atDesiredHeight(Math.abs(Carriage.elevatorOffSetPosition + Carriage.elevatorOffSetTolerace)) || forceWristRotate) {
                        wrist.setWristRotation(Carriage.wristOffSetPosition);
                    }
                    else if(!elevator.atDesiredHeight(Math.abs(Carriage.elevatorOffSetPosition + Carriage.elevatorOffSetTolerace))){
                        wrist.setWristRotation(Carriage.wristLiftPosition);
                        if(wrist.atDesiredRotation(Carriage.wristLiftPosition)) {
                            elevator.setElevatorHeight(Carriage.elevatorOffSetPosition);
                            if(elevator.atDesiredHeight(Math.abs(Carriage.elevatorOffSetPosition + Carriage.elevatorOffSetTolerace)) || forceWristRotate) {
                                wrist.setWristRotation(Carriage.wristOffSetPosition);
                            }
                        }
                    }                  
                }
            }

            case ReefL2 -> {
                intake.setState(IntakeStates.Transport);
                wrist.setWristRotation(Carriage.wristLiftPosition);
                if(wrist.atDesiredRotation(Carriage.wristLiftPosition)) {
                    elevator.setElevatorHeight(Carriage.elevatorL2Position);
                }
                if(elevator.atDesiredHeight(Carriage.elevatorL2Position,true) || forceWristRotate) {
                    wrist.setWristRotation(Carriage.wristL2L3Position);
                }
            }

            case ReefL3 -> {
                intake.setState(IntakeStates.Transport);
                wrist.setWristRotation(Carriage.wristLiftPosition);
                if(wrist.atDesiredRotation(Carriage.wristLiftPosition)) {
                    elevator.setElevatorHeight(Carriage.elevatorL3Position);
                }
                if(elevator.atDesiredHeight(Carriage.elevatorL3Position,true) || forceWristRotate) {
                    wrist.setWristRotation(Carriage.wristL2L3Position);
                }
            }

            case ReefL4 -> {
                intake.setState(IntakeStates.Transport);
                wrist.setWristRotation(Carriage.wristLiftPosition);
                if(wrist.atDesiredRotation(Carriage.wristLiftPosition)) {
                    elevator.setElevatorHeight(Carriage.elevatorL4Position);
                }  
                if(elevator.atDesiredHeight(Carriage.elevatorL4Position,true) || forceWristRotate) {
                    wrist.setWristRotation(Carriage.wristL4Position);
                }
            }

            case AlgaeL1 -> {
                wrist.setWristRotation(Carriage.wristAlgaeLiftPosition);
                if(wrist.atDesiredRotation(Carriage.wristAlgaeLiftPosition)) {
                    elevator.setElevatorHeight(Carriage.elevatorAlgaeL1Position);
                }  
                if(elevator.atDesiredHeight(Carriage.elevatorAlgaeL1Position,true) || forceWristRotate) {
                    wrist.setWristRotation(Carriage.wristAlgaePosition);
                }
            }

            case AlgaeL2 -> {
                wrist.setWristRotation(Carriage.wristLiftPosition);
                if(wrist.atDesiredRotation(Carriage.wristLiftPosition)) {
                    elevator.setElevatorHeight(Carriage.elevatorAlgaeL2Position);
                }  
                if(elevator.atDesiredHeight(Carriage.elevatorAlgaeL2Position,true) || forceWristRotate) {
                    wrist.setWristRotation(Carriage.wristAlgaePosition);
                }
            }

            case Net -> {
                wrist.setWristRotation(Carriage.wristAlgaeLiftPosition);
                elevator.setElevatorHeight(Carriage.elevatorNetPosition);
                if(elevator.atDesiredHeight(Carriage.elevatorNetPosition,true) || forceWristRotate) {
                    wrist.setWristRotation(Carriage.wristNetPosition);
                }
            }

            case Processor -> {
                wrist.setWristRotation(Carriage.wristAlgaeLiftPosition);
                if(wrist.atDesiredRotation(Carriage.wristAlgaeLiftPosition)) {
                    elevator.setElevatorHeight(Carriage.elevatorProcessorPosition);
                }  
                if(elevator.atDesiredHeight(Carriage.elevatorProcessorPosition) || forceWristRotate) {
                    wrist.setWristRotation(Carriage.wristProcessorPosition);
                    intake.setAlgaeTransport(true);
                }
            }

            case Climb -> {
                wrist.setWristRotation(Carriage.wristLiftPosition);
            }
            
            default -> {

            }
        }
    }
}
