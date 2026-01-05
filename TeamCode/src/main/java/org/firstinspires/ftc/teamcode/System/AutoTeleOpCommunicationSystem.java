package org.firstinspires.ftc.teamcode.System;

import static androidx.core.math.MathUtils.clamp;

import android.os.Environment;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;

@Configurable
public class AutoTeleOpCommunicationSystem {
    public static final String COMS_SAVE = PropertiesSystem.DATA_DIR + "/COMMUNICATION";


    //       _____   ______   _____   _____   _____   ______   ______   ___   _____   ______
    //      /  . /  /  .  /  /  . /  /  . /  /  __/  /  .  /  /_   _/  /  /  /  __/  /   __/
    //     /  __/  /    \   /  / /  /  __/  /  _/   /    \     / /    /  /  /  _/   /__   /
    //    /__/    /__/__/  /____/  /__/    /____/  /__/__/    /_/    /__/  /____/  /_____/
    // ---- PROPERTIES ---- //
    public enum Communicates {
        /**
         * The auto ID for the auto this should be unique to each auto
         ***/
        AUTO_ID,
        /**
         * Auto detected color of our alliance
         */
        AUTO_COLOR,
        /**
         * End pose positions so telelop knows where it starts for auto drive!
         */
        END_POSE_X,
        END_POSE_Y,
        END_POSE_RADIANS,
        /**
         * start Pos of Auto
         */
        START_POSE_X,
        START_POSE_Y,
        START_POSE_RADIANS
    }

    @IgnoreConfigurable
    public static final HashMap<Communicates, String> COMMUNICATABLES = new HashMap<Communicates, String>() {
        {
            //IMPLENTING V2 OF THE PROPERTIES SYSTEM!!!
            //NOW IF YOU SET THE VALUE FOR -1.0F OR -2.0F THOSE MEAN BOOLEAN VALUES CUSTOM VALUES SOOON
            //-20f means FORWARD and -21f means reverse
            //next is button asignment stuff for this so this can change button assignement!
            //-10f == zero power break -11f = float
            put(Communicates.AUTO_ID, "");
            put(Communicates.AUTO_COLOR, "");
            put(Communicates.END_POSE_X, "0.0");
            put(Communicates.END_POSE_Y, "0.0");
            put(Communicates.END_POSE_RADIANS, "0.0");
            put(Communicates.START_POSE_X, "0.0");
            put(Communicates.START_POSE_Y, "0.0");
            put(Communicates.START_POSE_RADIANS, "0.0");
//            put(Communicates.START_POSE_, "0.0");

//            put(Communicates.RIGHT_RUBBER_BAND_POWER, 0.5

        }
    };
    // END Properties

TelemetrySystem telemetry = null;
    public AutoTeleOpCommunicationSystem(){
//        try {
//            File dir = new File(DATA_SAVE);


    }


    public Communicates[] getCommunicationsSorted(){
        Communicates[] coms = COMMUNICATABLES.keySet().toArray(new Communicates[0]);

        for(int x = 0; x < coms.length - 1; x++){
            for(int y = 0; y < coms.length - 1; y++){
                if(coms[x].name().compareTo(coms[y].name()) > 0){
                    Communicates temp = coms[x];
                    coms[x] = coms[y];
                    coms[y] = temp;
                }
            }
        }

        return coms;
    }

    ///////////////////////////////////// OTHER RANDOM STUFF //////////////////////////////////////////

    /// Saves the log.

    public void saveCommunications() throws IOException {
        File f = new File(PropertiesSystem.DATA_DIR);
        if (!f.exists()) {
            f.mkdirs();
        }
        BufferedWriter wr = new BufferedWriter(new FileWriter(COMS_SAVE));
        COMMUNICATABLES.forEach((k, v) -> {
            try {
                wr.write(k.toString() + ":" + v + "\n");
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        });
        wr.close();
    }

    public static void loadCommunication() throws IOException {
        //                ArrayList<String> load_log = new ArrayList<>();
        BufferedReader rd = new BufferedReader(new FileReader(COMS_SAVE));
        while (true) {
            String line = rd.readLine();
            //                    load_log.add("Read Line: " + line);
            if (line == null) break;
            String[] split = line.split(":");
            String key = split[0];
            String value = split[1];
            try {
                try {
                    if (!COMMUNICATABLES.containsKey(Communicates.valueOf(key))) {
                        continue;
                    }
                    COMMUNICATABLES.put(Communicates.valueOf(key), value);
                } catch (IllegalArgumentException e) {
                    continue;
                }
            } catch (IndexOutOfBoundsException e) {
                continue;
            }
            //                    load_log.add("Key: " + key);
            //                    load_log.add("Value: " + value);
            //                    load_log.add("Enum: " + Prop.values()[key]);
        }
        //                telemetry.addLine("Finished loading; here's the log. Press 🔴 to continue.");
        //                for(String s : load_log){
        //                    telemetry.addLine(s);
        //                }
        //                telemetry.update();
        //                while(!gamepad1.circle || !gamepad2.circle);
    }

}