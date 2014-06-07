package com.cashlo.multiwiicam2gps.app;

import android.app.Activity;
import android.content.pm.ActivityInfo;
import android.os.Bundle;
import android.util.Log;
import android.view.GestureDetector;
import android.view.View;
import android.widget.RelativeLayout;

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


public class GPSStatusActivity extends Activity implements SampleApplicationControl {

    private static final String LOGTAG = "GPSStatus";
    private Renderer mRenderer;

    SampleApplicationSession vuforiaAppSession;

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

    boolean mIsDroidDevice = false;

    private final int latitudeHome = 523823900;
    private final int longitudeHome = 46429100;
    private final int altitudeHome    = 15;

    private MSP mMSP;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_gpsstatus);

        mMSP = new MSP(this);

        vuforiaAppSession = new SampleApplicationSession(this);
        mDatasetStrings.add("StonesAndChips.xml");
        mDatasetStrings.add("Tarmac.xml");

        vuforiaAppSession
                .initAR(this, ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
    }

    public void onLocationClick(View view) {

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

            int lat = latitudeAddMM(latitudeHome, (int)y);
            int lon = longitudeAddMM(latitudeHome, longitudeHome, (int)x);
            int altitude = altitudeHome + (int)(z/1000);


            Log.v("Location", "x:" + x + " y:" + y + " z:" + z);
            Log.v(LOGTAG, "Latitude: " + lat + " Longitude: " + lon + " Altitude: " + altitude);

            mMSP.writeGPS(lat, lon, altitude);
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
        if (tracker == null)
        {
            Log.e(
                    LOGTAG,
                    "Tracker not initialized. Tracker already initialized or the camera is already started");
            result = false;
        } else
        {
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
        for (int count = 0; count < numTrackables; count++)
        {
            Trackable trackable = mCurrentDataset.getTrackable(count);
            if(isExtendedTrackingActive())
            {
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

        if (mCurrentDataset != null && mCurrentDataset.isActive())
        {
            if (imageTracker.getActiveDataSet().equals(mCurrentDataset)
                    && !imageTracker.deactivateDataSet(mCurrentDataset))
            {
                result = false;
            } else if (!imageTracker.destroyDataSet(mCurrentDataset))
            {
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

        if (exception != null){
            Log.e(LOGTAG, exception.getString());
            finish();
        }

        try
        {
            vuforiaAppSession.startAR(CameraDevice.CAMERA.CAMERA_DEFAULT);
        } catch (SampleApplicationException e)
        {
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
        if (mSwitchDatasetAsap)
        {
            mSwitchDatasetAsap = false;
            TrackerManager tm = TrackerManager.getInstance();
            ImageTracker it = (ImageTracker) tm.getTracker(ImageTracker
                    .getClassType());
            if (it == null || mCurrentDataset == null
                    || it.getActiveDataSet() == null)
            {
                Log.d(LOGTAG, "Failed to swap datasets");
                return;
            }

            doUnloadTrackersData();
            doLoadTrackersData();
        }
    }

    public int latitudeAddMM (int latitudeStart, int distanceMM){
        int latitudeDifference = distanceMM * 10000000 / 11111111;

        Log.v(LOGTAG, "latitudeDifference: " + latitudeDifference);

        return latitudeStart + latitudeDifference ;
    }

    public int longitudeAddMM (int latitudeStart, int longitudeStart,  int distanceMM){
        int MM2degree =(int)( ((double)11111111) * Math.cos(Math.toRadians(latitudeStart / 10000000)));

        int longitudeDifference = distanceMM * 10000000 / MM2degree;
        Log.v(LOGTAG, "longitudeDifference: " + longitudeDifference);

        return longitudeStart + distanceMM * 10000000 / MM2degree;
    }

    boolean isExtendedTrackingActive()
    {
        return mExtendedTracking;
    }
}
