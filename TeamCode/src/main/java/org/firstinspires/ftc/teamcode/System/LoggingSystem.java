package org.firstinspires.ftc.teamcode.System;

import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOp26.telemetryS;

import android.util.Log;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;

public class LoggingSystem {
    PanelsSystem telemetryS = null;
    public LoggingSystem(PanelsSystem telemetry) {
        this.telemetryS = telemetry;
    }
    public final ArrayList<String> ACTION_LOG = new ArrayList<>();



    public static final String LOG_DIR = PropertiesSystem.DATA_DIR + "/LOGS";

    public LoggingSystem() {

    }
    public static final String DATA_SAVE = LOG_DIR + "/poseHistory.txt";

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
    public void savePose(Pose pose, Timer timer) throws IOException {
        File d = new File(LOG_DIR);
        if (!d.exists()) {
            d.mkdirs();
        }
        File f = new File(DATA_SAVE);
        if(!f.exists()) {
            telemetryS.speak("hiuhdshvhvkjvhvhxzkjhvvcxv");

            f.createNewFile();///Doesnt care if alread yexists
        }        Date time = Calendar.getInstance().getTime();
        BufferedWriter wr = new BufferedWriter(new FileWriter(f, true));

        wr.write(pose.getX() + ":" + pose.getY() + ":" + pose.getHeading() + "(ARG_TIME)" + time.getDate() + ";" + time.getHours() + ";" + time.getMinutes() + ";" + timer.getElapsedTimeSeconds() + "\n");

        wr.close();
    }
    public void savePose() throws IOException {
        File d = new File(LOG_DIR);
        if (!d.exists()) {
            d.mkdirs();
        }
        File f = new File(DATA_SAVE);
        if(!f.exists()) {
            telemetryS.speak("hiuhdshvhvkjvhvhxzkjhvvcxv");

            f.createNewFile();///Doesnt care if alread yexists
        }
        BufferedWriter wr = new BufferedWriter(new FileWriter(f, true));


        wr.close();
    }
    public void savePoses(ArrayList<Pose> poses, Timer timer) throws IOException {
        File d = new File(LOG_DIR);
        if (!d.exists()) {
            d.mkdirs();
        }
        File f = new File(DATA_SAVE);
        if(!f.exists()) {
            telemetryS.speak("hiuhdshvhvkjvhvhxzkjhvvcxv");

            f.createNewFile();///Doesnt care if alread yexists
        }
        BufferedWriter wr = new BufferedWriter(new FileWriter(f, true));
        Date time = Calendar.getInstance().getTime();

        for (Pose pose : poses) {
            wr.write(pose.getX() + ":" + pose.getY() + ":" + pose.getHeading() + "(ARG_TIME)" + time.getDate() + ";" + time.getHours() + ";" + time.getMinutes() + ";" + timer.getElapsedTimeSeconds() + "\n");
        }
        wr.close();
    }
    public void savePosesWithOverwrite(ArrayList<Pose> poses, Timer timer) throws IOException {
        File d = new File(LOG_DIR);
        if (!d.exists()) {
            d.mkdirs();
        }
        File f = new File(DATA_SAVE);
        if(!f.exists()) {
            telemetryS.speak("hiuhdshvhvkjvhvhxzkjhvvcxv");

            f.createNewFile();///Doesnt care if alread yexists
        }
        BufferedWriter wr = new BufferedWriter(new FileWriter(f));
        Date time = Calendar.getInstance().getTime();

        for (Pose pose : poses) {
            wr.write(pose.getX() + ":" + pose.getY() + ":" + pose.getHeading() + "(ARG_TIME)" + time.getDate() + ";" + time.getHours() + ";" + time.getMinutes() + ";" + timer.getElapsedTimeSeconds() + "\n");
        }
        wr.close();
    }
    public static ArrayList<Pose> loadPose() throws IOException {
                        ArrayList<Pose> loaded = new ArrayList<>();
        BufferedReader rd = new BufferedReader(new FileReader(DATA_SAVE));
        while (true) {
            String line = rd.readLine();
            //                    load_log.add("Read Line: " + line);
            if (line == null) break;
            ///  FORMAT X:Y:HEADING

            String[] split = line.split(":");
            try {
                double X = Double.parseDouble(split[0]);
                double Y = Double.parseDouble(split[1]);
                double HEADING = Double.parseDouble(split[2].split("(ARG_TIME)")[0].replace("(", ""));
                loaded.add(new Pose(X, Y, HEADING));
            } catch (Exception e) {
                //Jiust skip
                continue;
            }
//            float value = Float.parseFloat(split[1]);
//            try {
//                try {
//                    if (!PROPERTIES.containsKey(PropertiesSystem.Prop.valueOf(key))) {
//                        continue;
//                    }
//                    PROPERTIES.put(PropertiesSystem.Prop.valueOf(key), value);
//                } catch (IllegalArgumentException e) {
//                    continue;
//                }
//            } catch (IndexOutOfBoundsException e) {
//                continue;
//            }
            //                    load_log.add("Key: " + key);
            //                    load_log.add("Value: " + value);
            //                    load_log.add("Enum: " + Prop.values()[key]);
        }
        return loaded;
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
