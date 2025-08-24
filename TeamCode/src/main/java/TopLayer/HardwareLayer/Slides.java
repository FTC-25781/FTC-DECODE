package TopLayer.HardwareLayer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public interface Slides{

    void extend();

    void retract();

    void manualExtend();

    void manualRetract();

    void getPosition(int reference);

    void retractTo (double pos);

    void extendTo (double pos);

}
