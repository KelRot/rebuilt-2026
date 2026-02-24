package frc.robot.subsystems;

import static frc.robot.util.SparkUtil.ifOk;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.kicker.Kicker;
import frc.robot.subsystems.intake.Intake;


public class Superstructure extends SubsystemBase {

    private final Index index;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Kicker kicker;
    private final Hood hood;
    private final Turret turret;
    private final Drive drive;


    private SuperstructureState currentState = null;
    private SuperstructureState wantedState = null;

    public static enum SuperstructureState {
        OPENING_INTAKE,
        INTAKING,
        CLOSING_INTAKE,
        REJECTING_INTAKE,
        OUTTAKE,
        PREP_SHOOTING,
        SHOOTING,
        DEFAULT,
        TESTING,
        STOP,
        IDLE
    }

    public Superstructure(Intake intake, Flywheel flywheel, Kicker kicker, Hood hood, Turret turret, Drive drive, Index index) {
        this.intake = intake;
        this.flywheel = flywheel;
        this.kicker = kicker;
        this.hood = hood;
        this.turret = turret;
        this.drive = drive;
        this.index = index;
    }

    public void setWantedState(SuperstructureState state) {
        wantedState = state;
    }

    public SuperstructureState getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {
        currentState = wantedState; // ikisinin farklı olma sebebi bazı durumlarda wanted statein current statee
                                    // geçmeden önce bazı işlemler yapması gerekebilir (örneğin prep shooting
                                    // durumunda flywheel, hood ve turret istenilen pozisyona gelmeye çalışacak eğer
                                    // istenilen pozisyona gelirse shooting durumuna geçecek gibi) bu yüzden ikisi
                                    // farklı tutuluyor. Eğer böyle bir durum yoksa direkt olarak wanted statei
                                    // current state yapabiliriz.
        switch (currentState) {
            case OPENING_INTAKE:
                intake.requestState(Intake.SystemState.OPENING);
                break;

            case INTAKING:

            if(intake.isOpened()){
                intake.requestState(Intake.SystemState.INTAKING);
                index.requestState(Index.SystemState.INDEXING);
                 }

            else{
                break;
            }
                // Intake rollerları ve index dönücek diğer herhangi bir subsysteme dokunmayacak
                break;
            case CLOSING_INTAKE:
            if (!intake.isOpened()){
                intake.requestState(Intake.SystemState.CLOSING);
        }

                // Rollerlar durdurulucak intake kapanacak index durdurulacak diğer herhangi bir
                // subsysteme dokunmayacak
                break;
            case REJECTING_INTAKE:
            if (intake.isOpened()){
                intake.requestState(Intake.SystemState.OUTTAKING);
                // Rollerlar ters dönecek diğer herhangi bir subsysteme dokunmayacak ( Intake
                // kapalıysa çalışmayacak)
            }
                break;
            case OUTTAKE:
            if (!intake.isOpened()){
                intake.requestState(Intake.SystemState.OPENING);}
            
            index.requestState(Index.SystemState.OUTTAKING);
                // Intake kapalıysa açılacak rollerlar ters dönecek index de ters dönecek o
                // kadar diğer herhangi bir subsysteme dokunmayacak ( Intake zaten açıksa
                // rollerlar ters dönecek index de ters dönecek o kadar diğer herhangi bir
                // subsysteme dokunmayacak )
                // dokunmayacak
                break;
            case PREP_SHOOTING:

            hood.requestState(Hood.SystemState.POSITION);
            turret.requestState(Turret.SystemState.POSITION);
            flywheel.requestState(Flywheel.SystemState.TARGET_RPM);
                // Flywheel, hood ve turret istenilen pozisyona gelmeye çalışacak diğer herhangi
                // bir subsysteme dokunmayacak
                // Flywheel, hood ve turret istenilen pozisyona geldiğinde shooting stateine
                // geçilecek
            case SHOOTING:

            if(
                turret.isAtSetpoint()
            )
                // Kicker ekstra olarak aktif olacak diğer herhangi bir subsysteme dokunmayacak
                // ( Flywheel, hood ve turret istenilen pozisyonda kalmaya çalışacak )
                break;
            case DEFAULT:
                // Taret takip modunda olacak kicker dönmeyecek index dönmeyecek rollerlar
                // dönmeyecek intake kapalı olacak
                break;
            case TESTING:
                // her motor 1 voltla dönücek ( her motorun voltajı kendi constantsında
                // tekrardan tanımlanacak )
                break;
            case STOP:
                // Bütün motorlar durdurulacak herhangi bir subsysteme dokunulmayacak. ( Intake
                // acıksa kapanıcak )
                break;
            case IDLE:
                // hiç bir şey yapmayacak
                break;

        }
    }

}
