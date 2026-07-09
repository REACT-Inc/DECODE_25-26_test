package org.firstinspires.ftc.teamcode.System;

import com.bylazar.gamepad.GamepadManager;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;

//import com.byl
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Comparator;

public class PanelsSystem {
    static TelemetryManager telemetryM;
    static Telemetry telemetry;
    static GraphManager graph = PanelsGraph.INSTANCE.getManager();

    static GamepadManager PanelsPad1;
    static Gamepad gamepad1;

    static com.bylazar.gamepad.Gamepad PanelsPad2;
    static Gamepad gamepad2;

    public GamepadManager getGamepad2() {
        PanelsPad1 = PanelsGamepad.INSTANCE.getFirstManager();

        return PanelsPad1;
    }




    private class telemetryEntry {
        Class<?> section;
        Object type;
        Object[] data;

        public telemetryEntry(Class<?> cls, Object type, Object[] data) {
            this.section = cls;
            this.type = type;
            this.data = data;
        }
        public  telemetryEntry(Class<?> cls, Object line){
            this.section = cls;
            this.type = "addLine";
            this.data = new Object[]{line};
        }
        public boolean isSpeak(){
            return (type.toString() == "speak");
        }
        public boolean isData(){
            return (type.toString() == "addData");
        }
        public boolean isLine(){
            return (type.toString() == "addLine");
        }
    }
    public static ArrayList<String> sections = new ArrayList<String>();
    public static ArrayList<telemetryEntry> telemetryList = new ArrayList<>();
    public static ArrayList<telemetryEntry> telemetryManagerList = new ArrayList<>();


    public PanelsSystem(Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graph = PanelsGraph.INSTANCE.getManager();
        graph();

//    PanelsGamepad.INSTANCE.getFirstManager().getAsFTCGamepa

//    PanelsLights.INSTANCE.getLights().;
    }

    public  void addData(String one, String two) {
        telemetryM.addData(one, two);
//    telemetryM.update(telemetry);
    }

    public  void addData(Object one, Object two) {
        telemetryM.addData(one.toString(), two.toString());
//    telemetryM.update(telemetry);
    }

    public  void addLine(String one) {
        telemetryM.addLine(one);
//    telemetryM.update(telemetry);
    }

    public  void addLine() {
        telemetryM.addLine("");
//    telemetryM.update(telemetry);
    }

    public  void update() {
        graph.update();
        telemetryM.update(telemetry);
    }

    public  void speak(String t) {
        telemetry.speak(t);
    }

    public void addData(String upperSensorPressed, boolean pressed) {
        addData(upperSensorPressed, pressed + "");
    }

    public void addData(String upperSensorValue, double value) {
        addData(upperSensorValue, value + "");
    }

    public void clearAll() {
        telemetry.clearAll();
    }

    public void debug(String w, Object q) {
        telemetryM.debug(w, q.toString());
    }

    public void addData(String one, Object form, Object two) {
        telemetryM.addData(one, form.toString() + two.toString());
        telemetry.addData(one, form.toString(), two);
    }

    public void addData(String ll,  Object... cpu) {
        String nice = "";
        for(Object c : cpu){
            nice = nice + c.toString();
        }
        addData(ll, nice);
    }
    private Object[] merge(Object str, Object[] ary){
        Object[] rtn = new Object[ary.length + 1];
        int index = 0;
        for(Object a : ary){
            rtn[index] = a;
            index++;
        }
        rtn[index] = str;
        return rtn;
    }
    public void addTelemetryEntry(Class<?> cls, String txt, Object... Extras){
        //idk abot htis yet
        if(Extras.length == 0){

            telemetryList.add(new telemetryEntry(cls, txt));
            telemetryManagerList.add(new telemetryEntry(cls, txt));
        }else {
            telemetryList.add(new telemetryEntry(cls, "addData", merge(txt, Extras)));
            telemetryManagerList.add(new telemetryEntry(cls, "addData", merge(txt, Extras)));
        }
// Sorts by the nested section name
        telemetryList.sort(Comparator.comparing(obj -> obj.section.getName()));
        telemetryManagerList.sort(Comparator.comparing(obj -> obj.section.getName()));
    }

    public void newSectionBreak(Class<?> cls) {
        sections.add(cls.getName());
        addTelemetryEntry(cls, "-------------" + cls.getName() + "-------------");
    }


//    public void update(){
//        telemetry.so/
//    }
public void addGraphData(String txt, double value){
    graph.addData(txt, value);

}
public void graph(){
    graph.update();
    graph.getConfig().invoke().setGraphUpdateInterval(1);
    graph.getConfig().invoke().setEnabled(true);

}
}

//    public double GamepadRecieverDevice(){}