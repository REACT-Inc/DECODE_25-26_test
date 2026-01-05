package org.firstinspires.ftc.teamcode.System;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySystem {
    static TelemetryManager telemetryM;
    static Telemetry telemetry;
public TelemetrySystem(Telemetry telemetry){
    this.telemetry = telemetry;
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

}
public static void addData(String one, String two){
    telemetryM.addData(one,two);
//    telemetryM.update(telemetry);
}
    public static void addLine(String one ){
        telemetryM.addLine(one);
//    telemetryM.update(telemetry);
    }
    public static void addLine( ){
        telemetryM.addLine("");
//    telemetryM.update(telemetry);
    }
public static void update(){
    telemetryM.update(telemetry);
}
public static void speak(String t){
    telemetry.speak(t);
}

    public void addData(String upperSensorPressed, boolean pressed) {
    addData(upperSensorPressed, pressed + "");
    }

    public void addData(String upperSensorValue, double value) {
        addData(upperSensorValue, value + "");
    }
    public void clearAll(){
    telemetry.clearAll();
    }
    public void debug(String w, Object q){
    telemetryM.debug(w,q.toString());
    }
}
