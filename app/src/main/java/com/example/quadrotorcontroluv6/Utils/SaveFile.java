package com.example.quadrotorcontroluv6.Utils;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.res.AssetFileDescriptor;
import android.content.res.Resources;
import android.media.MediaPlayer;
import android.net.Uri;
import android.os.Environment;
import android.util.Log;
import android.widget.Toast;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.List;

import androidx.appcompat.app.AppCompatActivity;
import java.io.BufferedReader;
import java.io.FileReader;
import android.os.Bundle;

/**
 * Created by AAstudillo on 15/10/2017.
 */



public class SaveFile implements Serializable {

    private static Context ctx;
    private final Activity act;
    private File file, path;
    private int hour, minutes, seconds, year, month, day;

    private String file_name;
    private CharSequence app_name;
    public static MediaPlayer mp = new MediaPlayer();

    FileOutputStream fileOutputStream;
    DataOutputStream out;
    private FileWriter Info;

    private int minTimeOfData = 10; // in seconds

    String filenameArch = "";
    String filepath = "";
    public String fileContent = "";
    public String etInput;
    File myExternalFile;

    public int cont_guardado=0;

    public SaveFile (Context context, Activity activity){
        this.ctx = context;
        this.act= activity;
        file_name = "";
    }

    public void createFile(String filename){

        file_name = filename;
        Resources appR = ctx.getResources();
        app_name = appR.getText(appR.getIdentifier("app_name", "string", ctx.getPackageName()));

        hour = Calendar.getInstance().get(Calendar.HOUR_OF_DAY);
        minutes = Calendar.getInstance().get(Calendar.MINUTE);
        seconds = Calendar.getInstance().get(Calendar.SECOND);
        year = Calendar.getInstance().get(Calendar.YEAR);
        month = Calendar.getInstance().get(Calendar.MONTH) + 1;
        day = Calendar.getInstance().get(Calendar.DATE);

        //path = new File(Environment.getExternalStorageDirectory() + "/" + app_name + "/");"Teléfono" + "/" + "Download"
        filenameArch = "myFileQuad" +  "-" + year + "-" + month + "-" + day + "-" + hour + ":" + minutes + ":" + seconds + ".txt";
        filepath = "PruebasLPV";
        if (!isExternalStorageAvailableForRW()){
            //Toast.makeText(this.act,"No memory card",Toast.LENGTH_SHORT).show();
            Log.w("Revisión","No memory card");
        }

        myExternalFile = new File(ctx.getExternalFilesDir(filepath),filenameArch);
        Log.w("Nombre archivo","direccion archivo " + ctx.getExternalFilesDir(filepath));
        /*if (!path.exists()) {
            path.mkdir();
        }
        file = new File(path, file_name + "-" + year + "-" + month + "-" + day + "-" + hour + ":" + minutes + ":" + seconds + ".quv");

        try {
            Info = new FileWriter(file);
        } catch (IOException e) {
            e.printStackTrace();
        }*/



        //------------------------------------------------------------------------------------------


    }

    private boolean isExternalStorageAvailableForRW() {
        String extStorageState = Environment.getExternalStorageState();
        if (extStorageState.equals(Environment.MEDIA_MOUNTED)){
            return true;
        }
        return false;
    }

    public void writeDatainFile(String data){
        //"Pedro Luis Muñoz Murillo 34.5";
        etInput = etInput + data;

        if (cont_guardado==20) {
            //fileContent.concat(etInput.toString().trim());
            fileContent = etInput.toString().trim();//etInput.getText().toString().trim();
            if (!fileContent.equals("")) {
                //File myExternalFile = new File(ctx.getExternalFilesDir(filepath),filenameArch);
                FileOutputStream fos = null;
                try {
                    fos = new FileOutputStream(myExternalFile);
                    fos.write(fileContent.getBytes());
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }

                //Toast.makeText(this.act,"Information saved to SD card", Toast.LENGTH_SHORT).show();
                Log.w("Revisión archivo", "Information saved to SD card");
            } else {
                //Toast.makeText(this.act,"Text field no can be empty", Toast.LENGTH_SHORT).show();
                Log.w("Revisión archivo", "Text field no can be empty");
            }

            cont_guardado=0;
        }
        cont_guardado++;
        /*try {

            Log.w("Escribiendo","Escribiendo archivo");
            Info.append(data);
        } catch (IOException e) {
            e.printStackTrace();
        }*/

        /*dato = dato + data;
        text_Freq++;
        if (text_Freq == 6){
            try {
                Info.append(dato);
            } catch (IOException e) {
                e.printStackTrace();
            }
            text_Freq = 0;
            dato = " ";
        }*/
        Log.w("Revisión archivo","Escribiendo archivo");
    }

    public void closeFile(){
       /* Thread save = new Thread() {
            @Override
            public void run() {

                try {
                    Info.close();
                } catch (IOException e) {
                    e.printStackTrace();
                }
                    file.setReadable(true);
                    file.setWritable(true);

                Uri contentUri = Uri.fromFile(file);
                Intent mediaScanIntent = new Intent(Intent.ACTION_MEDIA_SCANNER_SCAN_FILE);
                mediaScanIntent.setData(contentUri);
                act.sendBroadcast(mediaScanIntent);

                try {
                    Thread.sleep(1500);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                playSound("donesaving");
            }
        };
        save.start();*/
        //Toast.makeText(this.act,"Information saved to SD card", Toast.LENGTH_SHORT).show();
        Log.w("Revisión archivo","Cerrando archivo");
        playSound("donesaving");
    }

    public void saveArrayList(final ArrayList<String> arrayList, String filename) {

        file_name = filename;

        final List<String> list = Collections.synchronizedList(arrayList);

        Resources appR = ctx.getResources();
        app_name = appR.getText(appR.getIdentifier("app_name", "string", ctx.getPackageName()));

        final int list_size = list.size();

        if(list_size >= minTimeOfData*100) { // data acquisition has a rate of approximately 100 times per second

            hour = Calendar.getInstance().get(Calendar.HOUR_OF_DAY);
            minutes = Calendar.getInstance().get(Calendar.MINUTE);
            seconds = Calendar.getInstance().get(Calendar.SECOND);
            year = Calendar.getInstance().get(Calendar.YEAR);
            month = Calendar.getInstance().get(Calendar.MONTH) + 1;
            day = Calendar.getInstance().get(Calendar.DATE);

            path = new File(Environment.getExternalStorageDirectory() + "/" + app_name + "/");
            if (!path.exists()) {
                path.mkdir();
            }

            file = new File(path, file_name + "-" + year + "-" + month + "-" + day + "-" + hour + ":" + minutes + ":" + seconds + ".quv");
            fileOutputStream = null;
            try {
                fileOutputStream = new FileOutputStream(file, true);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }

            out = new DataOutputStream(fileOutputStream);

            Thread save = new Thread() {
                @Override
                public void run() {
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    playSound("waittosave");
                    try {
                        StringBuilder sb = new StringBuilder();

                        synchronized (list) {

                            for (int j = 2; j <= list_size-2; j++) {
                                sb.append(list.get(j));
                            }
                            out.writeBytes(sb.toString());

                        }
                        out.close();
                        fileOutputStream.close();

                        file.setReadable(true);
                        file.setWritable(true);

                        Uri contentUri = Uri.fromFile(file);
                        Intent mediaScanIntent = new Intent(Intent.ACTION_MEDIA_SCANNER_SCAN_FILE);
                        mediaScanIntent.setData(contentUri);
                        act.sendBroadcast(mediaScanIntent);

                        playSound("donesaving");

                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            };

            save.start();
        }
    }

    public void saveStringBuilder(final StringBuilder sb, String filename) {

        file_name = filename;

        Resources appR = ctx.getResources();
        app_name = appR.getText(appR.getIdentifier("app_name", "string", ctx.getPackageName()));

        if(sb.length() > 100) {

            hour = Calendar.getInstance().get(Calendar.HOUR_OF_DAY);
            minutes = Calendar.getInstance().get(Calendar.MINUTE);
            seconds = Calendar.getInstance().get(Calendar.SECOND);
            year = Calendar.getInstance().get(Calendar.YEAR);
            month = Calendar.getInstance().get(Calendar.MONTH) + 1;
            day = Calendar.getInstance().get(Calendar.DATE);

            path = new File(Environment.getExternalStorageDirectory() + "/" + app_name + "/");
            if (!path.exists()) {
                path.mkdir();
            }

            file = new File(path, file_name + "-" + year + "-" + month + "-" + day + "-" + hour + ":" + minutes + ":" + seconds + ".quv");
            fileOutputStream = null;
            try {
                fileOutputStream = new FileOutputStream(file, true);
            } catch (FileNotFoundException e) {
                e.printStackTrace();
            }

            out = new DataOutputStream(fileOutputStream);

            Thread saveBuilder = new Thread() {
                @Override
                public void run() {
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    playSound("waittosave");
                    try {
                        out.writeBytes(sb.toString());
                        out.close();
                        fileOutputStream.close();

                        file.setReadable(true);
                        file.setWritable(true);

                        Uri contentUri = Uri.fromFile(file);
                        Intent mediaScanIntent = new Intent(Intent.ACTION_MEDIA_SCANNER_SCAN_FILE);
                        mediaScanIntent.setData(contentUri);
                        act.sendBroadcast(mediaScanIntent);

                        playSound("donesaving");

                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            };

            saveBuilder.start();
        }
    }

    private static void playSound(String sound){

        if(mp.isPlaying()) {
            mp.stop();
        }
        try {
            mp.reset();
            AssetFileDescriptor afd;
            afd = ctx.getAssets().openFd(sound+".mp3");
            mp.setDataSource(afd.getFileDescriptor(),afd.getStartOffset(),afd.getLength());
            mp.prepare();
            mp.start();
        } catch (IllegalStateException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
