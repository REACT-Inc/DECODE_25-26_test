package org.firstinspires.ftc.teamcode.System;
//package
import android.os.Environment;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class FileSystem {
    private final String COLUMN_SPLIT = ":IAMACOLUMNSPLIT:";
    private final String ROOT_DIR = Environment.getExternalStorageDirectory().getPath() + "/ACRobotics";
    private final String ARCHIVE_DIR = ROOT_DIR + "/Archived_Data_DcMotor";
    private String fileName;

    // Column Headers order for reconstruction
    private final String[] HEADERS = {
            "ConfigName", "RanUsingEncoder", "ReportedMotorRPM", "TestedTime",
            "StartingSpeed", "EndingSpeed", "MaxSpeedRecorded", "ARCTiming", "RampUpTime",
            "P", "F", "TrainingData", "Direction", "Date"
    };

    public FileSystem(String FileName) {
        this.fileName = FileName + ".txt";
        createDir(ROOT_DIR);
        createDir(ARCHIVE_DIR);
    }

    private void createDir(String path) {
        File f = new File(path);
        if (!f.exists()) f.mkdirs();
    }

    public boolean exists() {
        return new File(ROOT_DIR, fileName).exists();
    }

    public void archiveOldData() {
        File current = new File(ROOT_DIR, fileName);
        if (current.exists()) {
            String stamp = "_" + System.currentTimeMillis();
            File archive = new File(ARCHIVE_DIR, fileName.replace(".txt", "") + stamp + ".txt");
            current.renameTo(archive);
        }
    }

    public void writeData(Map<String, String> data) {
        try {
            BufferedWriter wr = new BufferedWriter(new FileWriter(new File(ROOT_DIR, fileName)));
            StringBuilder line = new StringBuilder();

            // Build the row based on HEADER order
            for (int i = 0; i < HEADERS.length; i++) {
                String key = HEADERS[i];
                String value = data.getOrDefault(key, "null");
                line.append(value);
                if (i < HEADERS.length - 1) {
                    line.append(COLUMN_SPLIT);
                }
            }

            wr.write(line.toString());
            wr.newLine(); // Row Split
            wr.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public Map<String, String> readData() {
        Map<String, String> result = new HashMap<>();
        try {
            BufferedReader rd = new BufferedReader(new FileReader(new File(ROOT_DIR, fileName)));
            String line = rd.readLine(); // Assume one row for motor config
            rd.close();

            if (line != null) {
                String[] parts = line.split(COLUMN_SPLIT);
                for (int i = 0; i < parts.length && i < HEADERS.length; i++) {
                    result.put(HEADERS[i], parts[i]);
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return result;
    }
}
