package utils;

import android.app.Activity;
import android.content.Context;
import android.os.Handler;
import android.util.Log;

import com.ftdi.j2xx.D2xxManager;
import com.ftdi.j2xx.FT_Device;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;


/**
 * Created by Cash on 07/06/2014.
 */
public class MSP {

    public static final int
            MSP_IDENT = 100,
            MSP_STATUS = 101,
            MSP_RAW_IMU = 102,
            MSP_SERVO = 103,
            MSP_MOTOR = 104,
            MSP_RC = 105,
            MSP_RAW_GPS = 106,
            MSP_COMP_GPS = 107,
            MSP_ATTITUDE = 108,
            MSP_ALTITUDE = 109,
            MSP_ANALOG = 110,
            MSP_RC_TUNING = 111,
            MSP_PID = 112,
            MSP_BOX = 113,
            MSP_MISC = 114,
            MSP_MOTOR_PINS = 115,
            MSP_BOXNAMES = 116,
            MSP_PIDNAMES = 117,
            MSP_SERVO_CONF = 120,


    MSP_SET_RAW_RC = 200,
            MSP_SET_RAW_GPS = 201,
            MSP_SET_PID = 202,
            MSP_SET_BOX = 203,
            MSP_SET_RC_TUNING = 204,
            MSP_ACC_CALIBRATION = 205,
            MSP_MAG_CALIBRATION = 206,
            MSP_SET_MISC = 207,
            MSP_RESET_CONF = 208,
            MSP_SELECT_SETTING = 210,
            MSP_SET_HEAD = 211, // Not used
            MSP_SET_SERVO_CONF = 212,
            MSP_SET_MOTOR = 214,


    MSP_BIND = 240,

    MSP_EEPROM_WRITE = 250,

    MSP_DEBUGMSG = 253,
            MSP_DEBUG = 254;
    public static final int
            IDLE = 0,
            HEADER_START = 1,
            HEADER_M = 2,
            HEADER_ARROW = 3,
            HEADER_SIZE = 4,
            HEADER_CMD = 5,
            HEADER_ERR = 6;
    int c_state = IDLE;
    static final int READBUF_SIZE  = 256;
    byte[] rbuf = new byte[READBUF_SIZE];
    char[] rchar = new char[READBUF_SIZE];
    static final int BAUDRATE = 115200;
    private static final String MSP_HEADER = "$M<";
    private static D2xxManager ftD2xx = null;
    private final String LOGTAG = "MSP";
    int mReadSize=0;
    boolean mThreadIsStopped = true;
    Handler mHandler = new Handler();
    Thread mThread;
    Context mContext;
    OnGPSWriteListener mListener;
    int byteRC_RATE, byteRC_EXPO, byteRollPitchRate, byteYawRate,
            byteDynThrPID, byteThrottle_EXPO, byteThrottle_MID, byteSelectSetting,
            cycleTime, i2cError,
            version, versionMisMatch, horizonInstrSize,
            GPS_distanceToHome, GPS_directionToHome,
            GPS_numSat, GPS_fix, GPS_update, GPS_altitude, GPS_speed,
            GPS_latitude, GPS_longitude,
            init_com, graph_on, pMeterSum, intPowerTrigger, bytevbat;
    boolean err_rcvd = false;
    byte checksum = 0;
    byte cmd;
    int offset = 0, dataSize = 0;
    byte[] inBuf = new byte[256];
    int p;
    int mode;
    boolean toggleRead = false, toggleReset = false, toggleCalibAcc = false, toggleCalibMag = false, toggleWrite = false,
            toggleRXbind = false, toggleSetSetting = false, toggleVbat = true, toggleMotor = false, motorcheck = true;
    int multiType, multiCapability = 0;
    private FT_Device ftDev;
    private Runnable mLoop = new Runnable() {
        @Override
        public void run() {
            int i;
            int readSize;
            mThreadIsStopped = false;
            while (true) {
                if (mThreadIsStopped) {
                    break;
                }

/*                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                }
*/
                synchronized (ftDev) {
                    readSize = ftDev.getQueueStatus();
                    if (readSize > 0) {
                        mReadSize = readSize;
                        if (mReadSize > READBUF_SIZE) {
                            mReadSize = READBUF_SIZE;
                        }
                        ftDev.read(rbuf, mReadSize);

                        decodeResponse(rbuf);

                    } // end of if(readSize>0)
                } // end of synchronized
            }
        }
    };
    private boolean mGPSWriting = false;
    private int present = 0;

    public MSP(Context context, OnGPSWriteListener listener) {
        mContext = context;
        mListener = listener;
        try {
            ftD2xx = D2xxManager.getInstance(context);
        } catch (D2xxManager.D2xxException ex) {
            Log.e(LOGTAG, ex.toString());
        }
    }

    public static List<Byte> requestMSP(int msp, Character[] payload) {
        if (msp < 0) {
            return null;
        }
        List<Byte> bf = new LinkedList<Byte>();
        for (byte c : MSP_HEADER.getBytes()) {
            bf.add(c);
        }


        byte checksum = 0;
        byte pl_size = (byte) ((payload != null ? payload.length : 0) & 0xFF);
        bf.add(pl_size);
        checksum ^= (pl_size & 0xFF);

        bf.add((byte) (msp & 0xFF));
        checksum ^= (msp & 0xFF);

        if (payload != null) {
            for (char c : payload) {
                bf.add((byte) (c & 0xFF));
                checksum ^= (c & 0xFF);
            }
        }
        bf.add(checksum);
        return (bf);
    }

    public static void sendRequestMSP(List<Byte> msp, FT_Device device) {
        byte[] arr = new byte[msp.size()];
        int i = 0;
        for (byte b : msp) {
            arr[i++] = b;
        }
        device.write(arr); // send the complete byte sequence in one go
    }

    static final public char parseChar(byte what) {
        return (char) (what & 0xff);
    };

    static final public char parseChar(int what) {
        return (char) what;
    }

    public int read32() {
        return (inBuf[p++] & 0xff) + ((inBuf[p++] & 0xff) << 8) + ((inBuf[p++] & 0xff) << 16) + ((inBuf[p++] & 0xff) << 24);
    }

    public int read16() {
        return (inBuf[p++] & 0xff) + ((inBuf[p++]) << 8);
    }

    public int read8() {
        return inBuf[p++] & 0xff;
    }

public synchronized void writeGPS(int latitude, int longitude, int altitude){
    openDevice();
    if (ftDev == null || ftDev.isOpen() == false) {
        Log.e(LOGTAG, "Device is not open");
        // mListener.onFailure();
        return;
        }

    ftDev.setLatencyTimer((byte) 16);
    List<Character> payload = new ArrayList<Character>();

    payload.add(MSP.parseChar(1)); //GPS_FIX
    payload.add(MSP.parseChar(8)); //GPS_numSat
    serialize32(payload, latitude);
    serialize32(payload, longitude);
    serialize16(payload, altitude);
    serialize16(payload, 0);
    serialize16(payload, 0);

    MSP.sendRequestMSP(MSP.requestMSP(MSP.MSP_SET_RAW_GPS, payload.toArray(new Character[payload.size()])), ftDev);
    }

    public synchronized void readGPS() {
        openDevice();
        if (ftDev == null || ftDev.isOpen() == false) {
            Log.e(LOGTAG, "Device is not open");
            // mListener.onFailure();
            return;
        }

        ftDev.setLatencyTimer((byte) 16);
        MSP.sendRequestMSP(MSP.requestMSP(MSP.MSP_RAW_GPS, null), ftDev);
    }

    private void serialize16(List<Character> payload, int a) {
        payload.add(parseChar((a) & 0xFF));
        payload.add(parseChar((a >> 8) & 0xFF));
    }

    private void serialize32(List<Character> payload, int a) {
        payload.add(parseChar((a) & 0xFF));
        payload.add(parseChar((a >> 8) & 0xFF));
        payload.add(parseChar((a >> 16) & 0xFF));
        payload.add(parseChar((a >> 24) & 0xFF));
    }

    private void openDevice() {
        if (ftDev != null) {
            if (ftDev.isOpen()) {
                if (mThreadIsStopped) {
                    SetConfig(BAUDRATE, (byte) 8, (byte) 1, (byte) 0, (byte) 0);
                    ftDev.purge((byte) (D2xxManager.FT_PURGE_TX | D2xxManager.FT_PURGE_RX));
                    ftDev.restartInTask();
                    new Thread(mLoop).start();
                }
                return;
            }
        }

        int devCount = 0;
        devCount = ftD2xx.createDeviceInfoList(mContext);

        Log.d(LOGTAG, "Device number : " + Integer.toString(devCount));

        D2xxManager.FtDeviceInfoListNode[] deviceList = new D2xxManager.FtDeviceInfoListNode[devCount];
        ftD2xx.getDeviceInfoList(devCount, deviceList);

        if (devCount <= 0) {
            return;
        }

        if (ftDev == null) {
            ftDev = ftD2xx.openByIndex(mContext, 0);
        } else {
            synchronized (ftDev) {
                ftDev = ftD2xx.openByIndex(mContext, 0);
            }
        }

        if (ftDev.isOpen()) {
            if (mThreadIsStopped) {
                SetConfig(115200, (byte) 8, (byte) 1, (byte) 0, (byte) 0);
                ftDev.purge((byte) (D2xxManager.FT_PURGE_TX | D2xxManager.FT_PURGE_RX));
                ftDev.restartInTask();
                new Thread(mLoop).start();
            }
        }
    }

    public void SetConfig(int baud, byte dataBits, byte stopBits, byte parity, byte flowControl) {
        if (ftDev.isOpen() == false) {
            Log.e(LOGTAG, "SetConfig: device not open");
            return;
        }

        // configure our port
        // reset to UART mode for 232 devices
        ftDev.setBitMode((byte) 0, D2xxManager.FT_BITMODE_RESET);

        ftDev.setBaudRate(baud);

        switch (dataBits) {
            case 7:
                dataBits = D2xxManager.FT_DATA_BITS_7;
                break;
            case 8:
                dataBits = D2xxManager.FT_DATA_BITS_8;
                break;
            default:
                dataBits = D2xxManager.FT_DATA_BITS_8;
                break;
        }

        switch (stopBits) {
            case 1:
                stopBits = D2xxManager.FT_STOP_BITS_1;
                break;
            case 2:
                stopBits = D2xxManager.FT_STOP_BITS_2;
                break;
            default:
                stopBits = D2xxManager.FT_STOP_BITS_1;
                break;
        }

        switch (parity) {
            case 0:
                parity = D2xxManager.FT_PARITY_NONE;
                break;
            case 1:
                parity = D2xxManager.FT_PARITY_ODD;
                break;
            case 2:
                parity = D2xxManager.FT_PARITY_EVEN;
                break;
            case 3:
                parity = D2xxManager.FT_PARITY_MARK;
                break;
            case 4:
                parity = D2xxManager.FT_PARITY_SPACE;
                break;
            default:
                parity = D2xxManager.FT_PARITY_NONE;
                break;
        }

        ftDev.setDataCharacteristics(dataBits, stopBits, parity);

        short flowCtrlSetting;
        switch (flowControl) {
            case 0:
                flowCtrlSetting = D2xxManager.FT_FLOW_NONE;
                break;
            case 1:
                flowCtrlSetting = D2xxManager.FT_FLOW_RTS_CTS;
                break;
            case 2:
                flowCtrlSetting = D2xxManager.FT_FLOW_DTR_DSR;
                break;
            case 3:
                flowCtrlSetting = D2xxManager.FT_FLOW_XON_XOFF;
                break;
            default:
                flowCtrlSetting = D2xxManager.FT_FLOW_NONE;
                break;
        }

        // TODO : flow ctrl: XOFF/XOM
        // TODO : flow ctrl: XOFF/XOM
        ftDev.setFlowControl(flowCtrlSetting, (byte) 0x0b, (byte) 0x0d);
    }

    public void decodeResponse(byte[] readByte) {
        for (byte b : readByte) {
            int c = b;
            if (c_state == IDLE) {
                c_state = (c == '$') ? HEADER_START : IDLE;
            } else if (c_state == HEADER_START) {
                c_state = (c == 'M') ? HEADER_M : IDLE;
            } else if (c_state == HEADER_M) {
                if (c == '>') {
                    c_state = HEADER_ARROW;
                } else if (c == '!') {
                    c_state = HEADER_ERR;
                } else {
                    c_state = IDLE;
                }
            } else if (c_state == HEADER_ARROW || c_state == HEADER_ERR) {
        /* is this an error message? */
                err_rcvd = (c_state == HEADER_ERR);        /* now we are expecting the payload size */
                dataSize = (c & 0xFF);
        /* reset index variables */
                p = 0;
                offset = 0;
                checksum = 0;
                checksum ^= (c & 0xFF);
        /* the command is to follow */
                c_state = HEADER_SIZE;
            } else if (c_state == HEADER_SIZE) {
                cmd = (byte) (c & 0xFF);
                checksum ^= (c & 0xFF);
                c_state = HEADER_CMD;
            } else if (c_state == HEADER_CMD && offset < dataSize) {
                checksum ^= (c & 0xFF);
                inBuf[offset++] = (byte) (c & 0xFF);
            } else if (c_state == HEADER_CMD && offset >= dataSize) {
        /* compare calculated and transferred checksum */
                if ((checksum & 0xFF) == (c & 0xFF)) {
                    if (err_rcvd) {
                        //System.err.println("Copter did not understand request type "+c);
                    } else {
            /* we got a valid response packet, evaluate it */
                        evaluateCommand(cmd, (int) dataSize);
                    }
                } else {
                    System.out.println("invalid checksum for command " + ((int) (cmd & 0xFF)) + ": " + (checksum & 0xFF) + " expected, got " + (int) (c & 0xFF));
                    System.out.print("<" + (cmd & 0xFF) + " " + (dataSize & 0xFF) + "> {");
                    for (int i = 0; i < dataSize; i++) {
                        if (i != 0) {
                            System.err.print(' ');
                        }
                        System.out.print((inBuf[i] & 0xFF));
                    }
                    System.out.println("} [" + c + "]");
                    System.out.println(new String(inBuf, 0, dataSize));
                }
                c_state = IDLE;
            }
        }

    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*
  static final public char parseChar(boolean what) {  // 0/1 or T/F ?
    return what ? 't' : 'f';
  }
  */

    public void evaluateCommand(byte cmd, int dataSize) {
        int i;
        int icmd = (int) (cmd & 0xFF);
        switch (icmd) {
            case MSP_IDENT:
                version = read8();
                multiType = read8();
                read8(); // MSP version
                multiCapability = read32();// capability
                break;

            case MSP_STATUS:
                cycleTime = read16();
                i2cError = read16();
                present = read16();
                mode = read32();
                break;
            case MSP_SET_RAW_GPS:
                //mListener.onSuccess();
                Log.v(LOGTAG, "GPS set successful!");
                break;
            case MSP_RAW_GPS:
                GPS_fix = read8();
                GPS_numSat = read8();
                GPS_latitude = read32();
                GPS_longitude = read32();
                GPS_altitude = read16();
                GPS_speed = read16();
                ((Activity) mContext).runOnUiThread(new Runnable() {
                    public void run() {
                        mListener.onGPSRead(GPS_latitude, GPS_longitude, GPS_altitude);
                    }
                });
                break;
            case MSP_COMP_GPS:
                GPS_distanceToHome = read16();
                GPS_directionToHome = read16();
                GPS_update = read8();
                break;
            default:
                Log.w(LOGTAG, "Don't know how to handle reply " + icmd);
        }
    }

    public interface OnGPSWriteListener {
        void onSuccess();

        void onFailure();

        void onGPSRead(int latitude, int longitude, int altitude);
    }

  /*
  static final public char parseChar(float what) {  // nonsensical
    return (char) what;
  }

  static final public char[] parseChar(String what) {  // note: array[]
    return what.toCharArray();
  }
  */

}
