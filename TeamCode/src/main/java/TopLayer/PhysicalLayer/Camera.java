package TopLayer.PhysicalLayer;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.content.Context;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.content.ContentValues;
import android.database.Cursor;

import java.util.List;

public class Camera {

    // Database helper class
    private static class AprilTagDBHelper extends SQLiteOpenHelper {
        private static final String DATABASE_NAME = "apriltags.db";
        private static final int DATABASE_VERSION = 1;
        private static final String TABLE_NAME = "detected_tags";
        private static final String COLUMN_ID = "id";
        private static final String COLUMN_TAG_ID = "tag_id";
        private static final String COLUMN_TIMESTAMP = "timestamp";
        private static final String COLUMN_X_POS = "x_position";
        private static final String COLUMN_Y_POS = "y_position";

        public AprilTagDBHelper(Context context) {
            super(context, DATABASE_NAME, null, DATABASE_VERSION);
        }

        @Override
        public void onCreate(SQLiteDatabase db) {
            String CREATE_TABLE = "CREATE TABLE " + TABLE_NAME + "("
                    + COLUMN_ID + " INTEGER PRIMARY KEY AUTOINCREMENT,"
                    + COLUMN_TAG_ID + " INTEGER,"
                    + COLUMN_TIMESTAMP + " INTEGER,"
                    + COLUMN_X_POS + " REAL,"
                    + COLUMN_Y_POS + " REAL" + ")";
            db.execSQL(CREATE_TABLE);
        }

        @Override
        public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
            db.execSQL("DROP TABLE IF EXISTS " + TABLE_NAME);
            onCreate(db);
        }
    }

    // Camera components
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDBHelper dbHelper;
    private SQLiteDatabase database;

    // Camera status
    private boolean isInitialized = false;
    private int lastDetectedTagId = -1;
    private long lastDetectionTime = 0;

    // Initialize camera with hardware map and context
    public Camera(HardwareMap hardwareMap, Context context) {
        // Initialize database
        dbHelper = new AprilTagDBHelper(context);
        database = dbHelper.getWritableDatabase();

        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)          // CPU efficient - no drawing
                .setDrawCubeProjection(false) // CPU efficient - no drawing
                .setDrawTagOutline(false)     // CPU efficient - no drawing
                .build();

        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        isInitialized = true;
    }

    // Process camera frame and detect AprilTags
    public void processFrame() {
        if (!isInitialized) return;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            int tagId = detection.id;
            long currentTime = System.currentTimeMillis();

            // Save to database
            saveTagDetection(tagId, currentTime, detection);

            // Update last detection info
            lastDetectedTagId = tagId;
            lastDetectionTime = currentTime;
        }
    }

    // Save AprilTag detection to SQLite database
    private void saveTagDetection(int tagId, long timestamp, AprilTagDetection detection) {
        ContentValues values = new ContentValues();
        values.put("tag_id", tagId);
        values.put("timestamp", timestamp);

        // Save position if available
        if (detection.ftcPose != null) {
            values.put("x_position", detection.ftcPose.x);
            values.put("y_position", detection.ftcPose.y);
        }

        database.insert("detected_tags", null, values);
    }

    // Get all detected tag IDs from database
    public int[] getAllDetectedTagIds() {
        Cursor cursor = database.query("detected_tags",
                new String[]{"DISTINCT tag_id"},
                null, null, null, null, "timestamp DESC");

        int[] tagIds = new int[cursor.getCount()];
        int index = 0;

        while (cursor.moveToNext()) {
            tagIds[index++] = cursor.getInt(0);
        }

        cursor.close();
        return tagIds;
    }

    // Get most recent tag ID
    public int getLastDetectedTagId() {
        return lastDetectedTagId;
    }

    // Get timestamp of last detection
    public long getLastDetectionTime() {
        return lastDetectionTime;
    }

    // Check if camera is operational
    public boolean isOperational() {
        return isInitialized && visionPortal != null &&
                visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    // Get current detections (for immediate use)
    public List<AprilTagDetection> getCurrentDetections() {
        if (!isInitialized) return null;
        return aprilTag.getDetections();
    }

    // Clear database
    public void clearDatabase() {
        database.execSQL("DELETE FROM detected_tags");
    }

    // Get detection count from database
    public int getDetectionCount() {
        Cursor cursor = database.rawQuery("SELECT COUNT(*) FROM detected_tags", null);
        cursor.moveToFirst();
        int count = cursor.getInt(0);
        cursor.close();
        return count;
    }

    // Get recent detections (last N seconds)
    public int[] getRecentDetections(int secondsBack) {
        long cutoffTime = System.currentTimeMillis() - (secondsBack * 1000L);

        Cursor cursor = database.query("detected_tags",
                new String[]{"DISTINCT tag_id"},
                "timestamp > ?",
                new String[]{String.valueOf(cutoffTime)},
                null, null, "timestamp DESC");

        int[] tagIds = new int[cursor.getCount()];
        int index = 0;

        while (cursor.moveToNext()) {
            tagIds[index++] = cursor.getInt(0);
        }

        cursor.close();
        return tagIds;
    }

    // Close camera and database
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
        if (database != null) {
            database.close();
        }
        if (dbHelper != null) {
            dbHelper.close();
        }
        isInitialized = false;
    }

    // Camera status for Physical Layer output
    public String getCameraStatus() {
        if (!isOperational()) {
            return "OFFLINE";
        } else if (lastDetectedTagId != -1) {
            return "DETECTING - Last ID: " + lastDetectedTagId;
        } else {
            return "ONLINE - No detections";
        }
    }
}
