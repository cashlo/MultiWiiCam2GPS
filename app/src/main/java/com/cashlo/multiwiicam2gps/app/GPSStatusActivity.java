package com.cashlo.multiwiicam2gps.app;

import android.app.Activity;
import android.content.Context;
import android.content.pm.ActivityInfo;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.util.Log;
import android.view.GestureDetector;
import android.view.View;
import android.widget.RelativeLayout;
import android.widget.TextView;

import com.qualcomm.vuforia.CameraDevice;
import com.qualcomm.vuforia.DataSet;
import com.qualcomm.vuforia.ImageTracker;
import com.qualcomm.vuforia.Matrix44F;
import com.qualcomm.vuforia.Renderer;
import com.qualcomm.vuforia.State;
import com.qualcomm.vuforia.Tool;
import com.qualcomm.vuforia.Trackable;
import com.qualcomm.vuforia.TrackableResult;
import com.qualcomm.vuforia.Tracker;
import com.qualcomm.vuforia.TrackerManager;

import java.util.ArrayList;

import utils.MSP;
import utils.SampleApplicationControl;
import utils.SampleApplicationException;
import utils.SampleApplicationSession;
import utils.SampleMath;


public class GPSStatusActivity extends Activity implements SampleApplicationControl, LocationListener, MSP.OnGPSWriteListener {

    private static final String LOGTAG = "GPSStatus";
    SampleApplicationSession vuforiaAppSession;
    private Renderer mRenderer;
    private DataSet mCurrentDataset;
    private int mCurrentDatasetSelectionIndex = 0;
    private int mStartDatasetsIndex = 0;
    private int mDatasetsNumber = 0;
    private ArrayList<String> mDatasetStrings = new ArrayList<String>();
    private GestureDetector mGestureDetector;
    private boolean mSwitchDatasetAsap = false;
    private boolean mFlash = false;
    private boolean mContAutofocus = false;
    private boolean mExtendedTracking = false;
    private View mFlashOptionView;
    private RelativeLayout mUILayout;
    private int latitudeHome;
    private int longitudeHome;
    private int altitudeHome;

    private boolean waitingLocation = false;

    private MSP mMSP;
    private LocationManager locationManager;
    private boolean stopCamera = false;
    private Runnable cameraLoop = new Runnable() {
        @Override
        public void run() {
            while (!stopCamera) {
                getCameraLocation();
            }
        }
    };
    private int writeCounter = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_gpsstatus);

        mMSP = new MSP(this, this);

        locationManager = (LocationManager)
                getSystemService(Context.LOCATION_SERVICE);

        Location location = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
        if (location != null) {
            latitudeHome = (int) (location.getLatitude() * 10000000);
            longitudeHome = (int) (location.getLongitude() * 10000000);
            altitudeHome = (int) location.getAltitude();
        } else {
            waitingLocation = true;
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, this);
        }

        vuforiaAppSession = new SampleApplicationSession(this);
        mDatasetStrings.add("StonesAndChips.xml");
        mDatasetStrings.add("Tarmac.xml");

        vuforiaAppSession
                .initAR(this, ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
    }

    public void onLocationChanged(Location location) {
        if (location != null) {
            latitudeHome = (int) (location.getLatitude() * 10000000);
            longitudeHome = (int) (location.getLongitude() * 10000000);
            altitudeHome = (int) location.getAltitude();
            waitingLocation = false;
            Log.v("Location Changed", location.getLatitude() + " and " + location.getLongitude());
            getCameraLocation();
            locationManager.removeUpdates(this);
        }
    }

    @Override
    public void onStatusChanged(String provider, int status, Bundle extras) {

    }

    @Override
    public void onProviderEnabled(String provider) {

    }

    @Override
    public void onProviderDisabled(String provider) {

    }

    public void onStopClick(View view) {
        stopCamera = true;
    }

    public synchronized void getCameraLocation() {
        //if (waitingLocation) {
        //    Log.e(LOGTAG, "Waiting for location");
        //    return;
        //}

        mRenderer = Renderer.getInstance();


        State state = mRenderer.begin();

        // did we find any trackables this frame?
        for (int tIdx = 0; tIdx < state.getNumTrackableResults(); tIdx++) {
            TrackableResult result = state.getTrackableResult(tIdx);
            Trackable trackable = result.getTrackable();
            Matrix44F modelViewMatrix_Vuforia = Tool
                    .convertPose2GLMatrix(result.getPose());


            Matrix44F inverseMV = SampleMath.Matrix44FInverse(modelViewMatrix_Vuforia);
            Matrix44F invTranspMV = SampleMath.Matrix44FTranspose(inverseMV);


            float[] camPose = invTranspMV.getData();


            float x = camPose[12];
            float y = camPose[13];
            float z = camPose[14];

            float cam_right_x = camPose[0];
            float cam_right_y = camPose[1];
            float cam_right_z = camPose[2];
            float cam_up_x = -camPose[4];
            float cam_up_y = -camPose[5];
            float cam_up_z = -camPose[6];
            float cam_dir_x = camPose[8];
            float cam_dir_y = camPose[9];
            float cam_dir_z = camPose[10];

            int lat = latitudeAddMM(latitudeHome, (int) y);
            int lon = longitudeAddMM(latitudeHome, longitudeHome, (int) x);
            int altitude = altitudeHome + (int) (z / 1000);


            Log.v("Location", "x:" + x + " y:" + y + " z:" + z);
            Log.v(LOGTAG, "Latitude: " + lat + " Longitude: " + lon + " Altitude: " + altitude);

            mMSP.writeGPS(lat, lon, altitude);
            writeCounter++;
            if (writeCounter > 10) {
                mMSP.readGPS();
                writeCounter = 0;
            }
        }
        mRenderer.end();
    }


    @Override
    public boolean doInitTrackers() {
        // Indicate if the trackers were initialized correctly
        boolean result = true;

        TrackerManager tManager = TrackerManager.getInstance();
        Tracker tracker;

        // Trying to initialize the image tracker
        tracker = tManager.initTracker(ImageTracker.getClassType());
        if (tracker == null) {
            Log.e(
                    LOGTAG,
                    "Tracker not initialized. Tracker already initialized or the camera is already started");
            result = false;
        } else {
            Log.i(LOGTAG, "Tracker successfully initialized");
        }
        return result;
    }

    @Override
    public boolean doLoadTrackersData() {
        TrackerManager tManager = TrackerManager.getInstance();
        ImageTracker imageTracker = (ImageTracker) tManager
                .getTracker(ImageTracker.getClassType());
        if (imageTracker == null)
            return false;

        if (mCurrentDataset == null)
            mCurrentDataset = imageTracker.createDataSet();

        if (mCurrentDataset == null)
            return false;

        if (!mCurrentDataset.load(
                mDatasetStrings.get(mCurrentDatasetSelectionIndex),
                DataSet.STORAGE_TYPE.STORAGE_APPRESOURCE))
            return false;

        if (!imageTracker.activateDataSet(mCurrentDataset))
            return false;

        int numTrackables = mCurrentDataset.getNumTrackables();
        for (int count = 0; count < numTrackables; count++) {
            Trackable trackable = mCurrentDataset.getTrackable(count);
            if (isExtendedTrackingActive()) {
                trackable.startExtendedTracking();
            }

            String name = "Current Dataset : " + trackable.getName();
            trackable.setUserData(name);
            Log.d(LOGTAG, "UserData:Set the following user data "
                    + (String) trackable.getUserData());
        }

        return true;
    }

    @Override
    public boolean doStartTrackers() {
        // Indicate if the trackers were started correctly
        boolean result = true;

        Tracker imageTracker = TrackerManager.getInstance().getTracker(
                ImageTracker.getClassType());
        if (imageTracker != null)
            imageTracker.start();

        return result;
    }

    @Override
    public boolean doStopTrackers() {
        // Indicate if the trackers were stopped correctly
        boolean result = true;

        Tracker imageTracker = TrackerManager.getInstance().getTracker(
                ImageTracker.getClassType());
        if (imageTracker != null)
            imageTracker.stop();

        return result;
    }

    @Override
    public boolean doUnloadTrackersData() {
        // Indicate if the trackers were unloaded correctly
        boolean result = true;

        TrackerManager tManager = TrackerManager.getInstance();
        ImageTracker imageTracker = (ImageTracker) tManager
                .getTracker(ImageTracker.getClassType());
        if (imageTracker == null)
            return false;

        if (mCurrentDataset != null && mCurrentDataset.isActive()) {
            if (imageTracker.getActiveDataSet().equals(mCurrentDataset)
                    && !imageTracker.deactivateDataSet(mCurrentDataset)) {
                result = false;
            } else if (!imageTracker.destroyDataSet(mCurrentDataset)) {
                result = false;
            }

            mCurrentDataset = null;
        }

        return result;
    }

    @Override
    public boolean doDeinitTrackers() {
        // Indicate if the trackers were deinitialized correctly
        boolean result = true;

        TrackerManager tManager = TrackerManager.getInstance();
        tManager.deinitTracker(ImageTracker.getClassType());

        return result;
    }

    @Override
    public void onInitARDone(SampleApplicationException exception) {

        if (exception != null) {
            Log.e(LOGTAG, exception.getString());
            finish();
        }

        try {
            vuforiaAppSession.startAR(CameraDevice.CAMERA.CAMERA_DEFAULT);
        } catch (SampleApplicationException e) {
            Log.e(LOGTAG, e.getString());
        }

        boolean result = CameraDevice.getInstance().setFocusMode(
                CameraDevice.FOCUS_MODE.FOCUS_MODE_CONTINUOUSAUTO);

        if (result)
            mContAutofocus = true;
        else
            Log.e(LOGTAG, "Unable to enable continuous autofocus");
    }

    @Override
    public void onQCARUpdate(State state) {
        if (mSwitchDatasetAsap) {
            mSwitchDatasetAsap = false;
            TrackerManager tm = TrackerManager.getInstance();
            ImageTracker it = (ImageTracker) tm.getTracker(ImageTracker
                    .getClassType());
            if (it == null || mCurrentDataset == null
                    || it.getActiveDataSet() == null) {
                Log.d(LOGTAG, "Failed to swap datasets");
                return;
            }

            doUnloadTrackersData();
            doLoadTrackersData();
        }
    }

    public int latitudeAddMM(int latitudeStart, int distanceMM) {
        int latitudeDifference = distanceMM * 10000000 / 11111111;

        Log.v(LOGTAG, "latitudeDifference: " + latitudeDifference);

        return latitudeStart + latitudeDifference;
    }

    public int longitudeAddMM(int latitudeStart, int longitudeStart, int distanceMM) {
        int MM2degree = (int) (((double) 11111111) * Math.cos(Math.toRadians(latitudeStart / 10000000)));

        int longitudeDifference = distanceMM * 10000000 / MM2degree;
        Log.v(LOGTAG, "longitudeDifference: " + longitudeDifference);

        return longitudeStart + distanceMM * 10000000 / MM2degree;
    }

    boolean isExtendedTrackingActive() {
        return mExtendedTracking;
    }

    @Override
    public void onSuccess() {
        //if (!stopCamera) {
        //   getCameraLocation();
        //}
    }

    @Override
    public void onFailure() {
        //if (!stopCamera) {
        //     new Timer().schedule(new TimerTask() {
        //         @Override
        //        public void run() {
        //            getCameraLocation();
        //        }
        //    }, 500);
        // }
    }

    @Override
    public void onGPSRead(int latitude, int longitude, int altitude) {
        TextView GPSStatus = (TextView) findViewById(R.id.GPS_status);
        GPSStatus.setText("latitude: " + latitude + " longitude: " + longitude + " altitude: " + altitude);
    }

    public void onStartClick(View view) {
        stopCamera = false;
        new Thread(cameraLoop).start();
    }
}
