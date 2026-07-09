package org.firstinspires.ftc.teamcode.System;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;

public class AllianceDependentObject {
    PanelsSystem telemetryS = null;
    public AllianceDependentObject(PanelsSystem telemetry) {
        this.telemetryS = telemetry;
    }
    public final ArrayList<String> ACTION_LOG = new ArrayList<>();



    public final String LOG_DIR = PropertiesSystem.DATA_DIR + "/LOGS";

    public AllianceDependentObject() {

    }

    public void saveLog() throws IOException {
        File d = new File(LOG_DIR);
        if (!d.exists()) {
            d.mkdirs();
        }
        File f = new File(LOG_DIR + "/" + new SimpleDateFormat("dd-MM-yyyy_HHmmss").format(Calendar.getInstance().getTime()).toString() + ".txt");
        f.createNewFile();
        BufferedWriter wr = new BufferedWriter(new FileWriter(f));
        for (String l : ACTION_LOG) {
            wr.write(l + "\n");
        }
        wr.close();
    }
    //Loop function for showing program log
    public void solog() {
        for (int i = ACTION_LOG.size() - 1; i >= 0; i--) {
            telemetryS.addLine(ACTION_LOG.get(i));
        }
        telemetryS.update();
    }
    public void log(String info) {
        info = "[" + Calendar.getInstance().get(Calendar.HOUR) + ":" + Calendar.getInstance().get(Calendar.MINUTE) + ":" + Calendar.getInstance().get(Calendar.SECOND) + " (" + " seconds in)] " + info;
        ACTION_LOG.add(info);
    }

    public void setTelemetry(PanelsSystem telemetryS) {
        this.telemetryS = telemetryS;
    }
}
