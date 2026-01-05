package org.firstinspires.ftc.teamcode.System;

import static androidx.core.math.MathUtils.clamp;

import android.os.Environment;
import android.os.FileUtils;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.*;
import java.nio.file.FileStore;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.util.*;

import javax.tools.FileObject;
@Configurable
public class PropertiesSystem {
    @IgnoreConfigurable
    public static final String DATA_DIR = Environment.getExternalStorageDirectory().getPath() + "/ATEAM";
    @IgnoreConfigurable
    public static final String DATA_SAVE = DATA_DIR + "/PROPERTIES";


    //       _____   ______   _____   _____   _____   ______   ______   ___   _____   ______
    //      /  . /  /  .  /  /  . /  /  . /  /  __/  /  .  /  /_   _/  /  /  /  __/  /   __/
    //     /  __/  /    \   /  / /  /  __/  /  _/   /    \     / /    /  /  /  _/   /__   /
    //    /__/    /__/__/  /____/  /__/    /____/  /__/__/    /_/    /__/  /____/  /_____/
    // ---- PROPERTIES ---- //
    public enum Prop {
        /**
         * The speed the horizontal slides move at.
         ***/
        RIGHT_WRIST_POWER,
        /**
         * The amount of samples used in averaging Limelight results.
         ***/
        LEFT_WRIST_POWER,
        /**
         * The multiplier applied to the speed boost.
         **/
        RIGHT_RUBBER_BAND_POWER,
        /**
         * The multiplier applied to the speed dampening.
         **/
        LEFT_RUBBER_BAND_POWER,
        /**
         * The lower position of the left alignment servo.
         **/
        MID_INTAKE_POWER,
        /**
         * The upper position of the left alignment servo.
         **/
        LED_COUNT_PRISM_DRIVER,
        /**
         * The upper position of the right alignment servo.
         **/
        UPPER_MAGNETIC_SENSOR_ENABLED,
        /**
         * The speed he crane lifts and drops at.
         **/
        LOWER_MAGNETIC_SENSOR_ENABLED,
        /**
         * The position of the gripper when it's open.
         **/
        BACK_MOTOR_SPEED,
        /**
         * The position of the gripper when it's closed.
         **/
        LAUNCHER_MOTOR_PIDF_P_VALUE,
        /**
         * The color of the PWM light when the gripper is open.
         **/
        LAUNCHER_MOTOR_PIDF_I_VALUE,
        /**
         * The color of the PWM light when the gripper is closed.
         **/
        LAUNCHER_MOTOR_PIDF_D_VALUE,
        /**
         * The max position of the slide.
         **/
        LAUNCHER_MOTOR_PIDF_F_VALUE,
        /**
         * The wrist's max pos
         */
        FL_MOTOR_DIRECTION,
        /**
         * The wrist's min pos
         */
        FR_MOTOR_DIRECTION,
        /**
         * How much the wrist's speed is dampened by.
         */
        BL_MOTOR_DIRECTION,
        /**
         * A multiplier for the vertical slide's speed when L1 is pressed on gamepad2.
         */
        BR_MOTOR_DIRECTION,
        /**
         * The amount to increment the servo by to rotate 45 degrees.
         */
        DRIVE_MOTOR_ZERO_POWER_BHV,
        LED_BRIGHTNESS_PRISM_DRIVER,
        LED_SPEED_PRISM_DRIVER,
        /**
         * Controls if drivers can interact with controls while in menu the menu doesnt need gamepad 2 so while enabled
         * Driver 2 controls all states of the robot
         */
        TOP_ALLOW_CONTROLS_IN_MENU,
        /**
         * This controls if the Leds are enabled or not
         */
        LED_ENABLED_PRISM_DRIVER,
        /**
         * This can disable controller drive (IF WE HAVE IT ON A TABLE THIS IS EPIC!)
         */
        CONTROLLER_DRIVE_ENABLED,
        /**
         * Disableds auto driveing things
         */
        AUTO_DRIVE_DISABLED
    }

    @IgnoreConfigurable
    public static final HashMap<Prop, Float> PROPERTIES = new HashMap<Prop, Float>() {
        {
            //IMPLENTING V2 OF THE PROPERTIES SYSTEM!!!
            //NOW IF YOU SET THE VALUE FOR -1.0F OR -2.0F THOSE MEAN BOOLEAN VALUES CUSTOM VALUES SOOON
            //-20f means FORWARD and -21f means reverse
            //next is button asignment stuff for this so this can change button assignement!
            //-10f == zero power break -11f = float
            put(Prop.RIGHT_WRIST_POWER, 0.5f);
            put(Prop.LEFT_WRIST_POWER, 0.5f);
            put(Prop.RIGHT_RUBBER_BAND_POWER, 0.5f);
            put(Prop.LEFT_RUBBER_BAND_POWER, 0.5f);
            put(Prop.MID_INTAKE_POWER, 1f);
            put(Prop.LED_COUNT_PRISM_DRIVER, 48f);
            put(Prop.UPPER_MAGNETIC_SENSOR_ENABLED, -1f);
            put(Prop.LOWER_MAGNETIC_SENSOR_ENABLED, -1f);
            put(Prop.BACK_MOTOR_SPEED, 1f);
            put(Prop.LAUNCHER_MOTOR_PIDF_P_VALUE, 0.0f);
            put(Prop.LAUNCHER_MOTOR_PIDF_I_VALUE, 0.3f);
            put(Prop.LAUNCHER_MOTOR_PIDF_D_VALUE, 0.28f);
            put(Prop.LAUNCHER_MOTOR_PIDF_F_VALUE, 0f);
            put(Prop.FL_MOTOR_DIRECTION, -21f);
            put(Prop.FR_MOTOR_DIRECTION, -20f);
            put(Prop.BL_MOTOR_DIRECTION, -21f);
            put(Prop.DRIVE_MOTOR_ZERO_POWER_BHV, -10f);
            put(Prop.BR_MOTOR_DIRECTION, -20f);
            put(Prop.LED_BRIGHTNESS_PRISM_DRIVER, -100f);
            put(Prop.LED_SPEED_PRISM_DRIVER, 100f);
            put(Prop.TOP_ALLOW_CONTROLS_IN_MENU, -1f);
            put(Prop.LED_ENABLED_PRISM_DRIVER, -1f);
            put(Prop.CONTROLLER_DRIVE_ENABLED, -1f);
            put(Prop.AUTO_DRIVE_DISABLED, -1f);

        }
    };
    // END Properties

    public boolean adjustmentMode = false;
TelemetrySystem telemetry = null;
    public int adjustmentIndex = 0;
    public Prop adjust_selected = null;
    public final int UI_BTN_PRESS_DELAY = 40;

    public final float[] boolean_numbers = {-1f, -2f};
    public final float[] zero_power_numbers = {-10f, -11f};
    public final float[] direction_numbers = {-20f, -21f};
    public final boolean[] boolean_values = {true, false};
    public final String[] zero_power_values = {"BRAKE", "FLOAT"};
    public final String[] direction_values = {"FORWARD", "REVERSE"};
    public final char[] unselected_numbers = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '.', '-'};
    public  final String[] selected_numbers = {"1️⃣", "2️⃣", "3️⃣", "4️⃣", "5️⃣", "6️⃣", "7️⃣", "8️⃣", "9️⃣", "0️⃣", "⊡", "➖"};
    public  int adjust_keypadIndex = 0;
    public String adjust_input = "REPLACE";
    public float[] setInUse;// = null;
public boolean  haveUnderscore = true;
    public  PropertiesSystem(TelemetrySystem telemetry){
        this.telemetry = telemetry;
//        try {
//            File dir = new File(DATA_SAVE);


    }
    public void t(){
//        PROPERTIES.get(Prop.HS_STARTING_POS);
    }
    public void startMenu(Gamepad gamepad1) {
        telemetry.addLine("Adjustment Mode");
        telemetry.addLine("_______________");
        telemetry.addLine();
        if (adjust_selected == null) {
            //            List<Map.Entry<Prop, Float>> sortedEntries = new ArrayList<>(PROPERTIES.entrySet());
            //            sortedEntries.sort(Comparator.comparing(e -> e.getKey().name())); // DOESN'T WORK

            Prop current = null;
            int i = 0;
            for (Prop k : getPropertiesSorted()) {
                Object v = PropertyCleaner(PROPERTIES.get(k));
                telemetry.addLine((adjustmentIndex == i ? ">" : " ") + k.toString() + ": " + v.toString());
                if (i == adjustmentIndex) {
                    current = k;
                }
                i++;
            }
            telemetry.addLine("_______________");
            telemetry.addLine();
            telemetry.addLine("↕ Navigate    ✖ Select    ≡ Exit");

            //sleep(UI_BTN_PRESS_DELAY);
            if (gamepad1.dpad_up) {
                adjustmentIndex--;
                while (gamepad1.dpad_up) ;
            } else if (gamepad1.dpad_down) {
                adjustmentIndex++;
                while (gamepad1.dpad_down) ;
            } else if (gamepad1.cross) {
                adjust_selected = current;
                adjust_input = "REPLACE";
                adjust_keypadIndex = 0;
                while (gamepad1.cross) ;
            }
        } else {
            boolean cloningAllowed = false;
//            if (adjust_selected == Prop.PRESET_CIRCLE_VS ||
//                    adjust_selected == Prop.PRESET_CROSS_VS ||
//                    adjust_selected == Prop.PRESET_SQUARE_VS ||
//                    adjust_selected == Prop.PRESET_TRIANGLE_VS) {
//                cloningAllowed = true;
//            }
            telemetry.addLine("Editing: " + adjust_selected);
            telemetry.addLine();
            telemetry.addLine("Enter Value:");
            haveUnderscore = !haveUnderscore;
            String adjusted_input = null;
            if(adjust_input == "REPLACE"){
                adjust_input = (PROPERTIES.get(adjust_selected)).toString();
            }

            for (int i = 0; i < boolean_numbers.length; i++) {
                if (boolean_numbers[i] == Float.parseFloat(adjust_input)) {
                    adjusted_input = boolean_values[i] + "";// += selected_numbers[i];
                    break;
                }
            }
            if(adjusted_input == null) {
                for (int i = 0; i < zero_power_numbers.length; i++) {
                    if (zero_power_numbers[i] == Float.parseFloat(adjust_input)) {
                        adjusted_input = zero_power_values[i] + "";// += selected_numbers[i];
                        break;
                    }
                }
            }
            if(adjusted_input == null) {
                for (int i = 0; i < direction_numbers.length; i++) {
                    if (direction_numbers[i] == Float.parseFloat(adjust_input)) {
                        adjusted_input = direction_values[i] + "";// += selected_numbers[i];
                        break;
                    }
                }
            }
            if(adjusted_input == null) {
                adjusted_input = adjust_input;
            }
            telemetry.addLine(adjusted_input  + ((haveUnderscore == true) ? "" : "_"));
            telemetry.addLine();
            String keypad = "";
            for (int i = 0; i < selected_numbers.length; i++) {
                keypad += " ";
                if (i == adjust_keypadIndex) {
                    keypad += selected_numbers[i];
                } else {
                    keypad += unselected_numbers[i];
                }
            }
            Object tested = PropertyCleaner(PROPERTIES.get(adjust_selected));
            if(tested instanceof Float){
            }else{
                float f = PROPERTIES.get(adjust_selected);
                if(f == boolean_numbers[0] || f == boolean_numbers[1]){
                    keypad = "";
//                    keypad = boolean_values[0] + " " + boolean_values[1];
                    for (int i = 0; i < boolean_values.length; i++) {
                        keypad += " ";
                        if (i == adjust_keypadIndex) {
                            keypad += boolean_values[i];
                        } else {
                            keypad += boolean_values[i];
                        }
                    }
                    setInUse = boolean_numbers;
                }
                if(f == zero_power_numbers[0] || f == zero_power_numbers[1]){
                    keypad = "";
//                    keypad = zero_power_values[0] + " " + zero_power_values[1];
                    for (int i = 0; i < zero_power_values.length; i++) {
                        keypad += " ";
                        if (i == adjust_keypadIndex) {
                            keypad += zero_power_values[i];
                        } else {
                            keypad += zero_power_values[i];
                        }
                    }
                    setInUse = zero_power_numbers;
                }
                if(f == direction_numbers[0] || f == direction_numbers[1]){
                    keypad = "";
//                    keypad = direction_values[0] + " " + direction_values[1];
                    for (int i = 0; i < direction_values.length; i++) {
                        keypad += " ";
                        if (i == adjust_keypadIndex) {
                            keypad += direction_values[i];
                        } else {
                            keypad += direction_values[i];
                        }
                    }
                    setInUse = direction_numbers;
                }
            }
            telemetry.addLine(keypad);
            telemetry.addLine("_______________");
            telemetry.addLine("↔ Navigate    ❌ Select    🔺 Backspace    🔴 Return" + (cloningAllowed ? "    🟥 Clone from Device" : ""));

            if (gamepad1.dpad_left) {
                adjust_keypadIndex--;
                if(setInUse == null) {
//                    adjust_input += unselected_numbers[adjust_keypadIndex];
                }else{
                    if(setInUse.length > adjust_keypadIndex && -1 < adjust_keypadIndex) {
                        adjust_input = setInUse[adjust_keypadIndex] + "";
                    }
                }
                while (gamepad1.dpad_left) ;
            } else if (gamepad1.dpad_right) {
                adjust_keypadIndex++;
                if(setInUse == null) {
//                    adjust_input += unselected_numbers[adjust_keypadIndex];
                }else{
                    if(setInUse.length > adjust_keypadIndex && -1 < adjust_keypadIndex) {
                        adjust_input = setInUse[adjust_keypadIndex] + "";
                    }                }
                while (gamepad1.dpad_right) ;
            } else if (gamepad1.circle) {
                PROPERTIES.put(adjust_selected, Float.parseFloat(adjust_input));
                adjust_selected = null;
                adjust_input = "";
                adjust_keypadIndex = 0;
                while (gamepad1.circle) ;
            } else if (gamepad1.cross) {
                if(setInUse == null) {
                    adjust_input += unselected_numbers[adjust_keypadIndex];
                }else{
                    adjust_input = setInUse[adjust_keypadIndex] + "";
                }
                while (gamepad1.cross) ;
            } else if (gamepad1.triangle) {
                adjust_input = adjust_input.substring(0, adjust_input.length() - 1);
                while (gamepad1.triangle) ;
            } //else if (gamepad1.square) {
//                if (cloningAllowed) {
//                    if (adjust_selected == Prop.PRESET_CIRCLE_VS ||
//                            adjust_selected == Prop.PRESET_CROSS_VS ||
//                            adjust_selected == Prop.PRESET_SQUARE_VS ||
//                            adjust_selected == Prop.PRESET_TRIANGLE_VS) {
//                        adjust_input = String.valueOf(VERTICAL_SLIDE_1.getCurrentPosition());
//                    }
//                }
//            }
        }
        telemetry.update();

        adjustmentIndex = (int) clamp(adjustmentIndex, 0, PROPERTIES.size());
    }

    public Prop[] getPropertiesSorted(){
        Prop[] props = PROPERTIES.keySet().toArray(new Prop[0]);

        for(int x = 0; x < props.length - 1; x++){
            for(int y = 0; y < props.length - 1; y++){
                if(props[x].name().compareTo(props[y].name()) > 0){
                    Prop temp = props[x];
                    props[x] = props[y];
                    props[y] = temp;
                }
            }
        }

        return props;
    }

    ///////////////////////////////////// OTHER RANDOM STUFF //////////////////////////////////////////

    /// Saves the log.

    public void saveProperties() throws IOException {
        File f = new File(DATA_DIR);
        if (!f.exists()) {
            f.mkdirs();
        }
        BufferedWriter wr = new BufferedWriter(new FileWriter(DATA_SAVE));
        PROPERTIES.forEach((k, v) -> {
            try {
                wr.write(k.toString() + ":" + v + "\n");
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        });
        wr.close();
    }

    public static void loadProperties() throws IOException {
        //                ArrayList<String> load_log = new ArrayList<>();
        BufferedReader rd = new BufferedReader(new FileReader(DATA_SAVE));
        while (true) {
            String line = rd.readLine();
            //                    load_log.add("Read Line: " + line);
            if (line == null) break;
            String[] split = line.split(":");
            String key = split[0];
            float value = Float.parseFloat(split[1]);
            try {
                try {
                    if (!PROPERTIES.containsKey(Prop.valueOf(key))) {
                        continue;
                    }
                    PROPERTIES.put(Prop.valueOf(key), value);
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
public static Object PropertyCleaner(float value){
        if(value == -1f){
            return true;
        }else if(value == -2f) {
            return false;
        }else if(value == -20f){
            return "FORWARD";
        }else if(value == -21f){
            return "REVERSE";

        }else if(value == -10f) {
            return "BRAKE";
        }else if(value == -11f){
            return "FLOAT";
        }else{
            return value;
        }
}


public static String buttonRegex(Gamepad gamepad1, Gamepad gamepad2){

    return "";
}
}