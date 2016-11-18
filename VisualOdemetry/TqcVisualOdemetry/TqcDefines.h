#ifndef __TQC_DEFINES_H
#define __TQC_DEFINES_H

#define TQC_DEBUG                    1    // Enable debug code.

#define TQC_Z_SCALE 10.0

#define TQC_POSE_VERTEX_COLOR        0.5f, 0.5f, 0.8f
#define TQC_POSE_LATEST_VERTEX_COLOR 1.0f, 0.0f, 0.0f
#define TQC_LOADED_POSE_COLOR1       0.4f, 0.8f, 0.0f
#define TQC_LOADED_POSE_COLOR2       0.8f, 0.8f, 0.0f
#define TQC_LOADED_POSE_COLOR3       0.0f, 0.8f, 0.4f
#define TQC_LOADED_POSE_COLOR4       0.0f, 0.8f, 0.8f
#define TQC_LOADED_POSE_COLOR5       0.8f, 0.0f, 0.4f
#define TQC_POSE_TEXT_COLOR          0.8f, 0.0f, 0.0f
#define TQC_POSE_SELECTED_COLOR      0.0f, 0.0f, 0.8f
#define TQC_POSE_COLOR_NUM           7
#define TQC_POSE_COLOR_VIDEO_INDEX   0
#define TQC_POSE_COLOR_LOAD_INDEX    2

#define TQC_POSE_EDGE_WIDTH  5
#define TQC_POSE_SCALE       1
#define TQC_POSE_ARROW_SCALE 5
#define TQC_POSE_ARROW_WIDTH 0.01 * 5
#define TQC_POSE_ARROW_LEN   0.012 * 5

#define TQC_VIDEO_POSES_ARRAY_NUM  1
#define TQC_LOAD_POSES_ARRAY_NUM   5
#define TQC_TOTAL_POSES_ARRAY_NUM  TQC_VIDEO_POSES_ARRAY_NUM + TQC_LOAD_POSES_ARRAY_NUM
#define TQC_VIDEO_POSE_ARRAY_INDEX 0
#define TQC_LOAD_POSES_ARRAY_START 1
#define TQC_LOAD_POSES_ARRAY_END   5

// 3D View
#define TQC_VIEW_CAM_DIST 1.0              // default camera distance

// Bundle Adjustment
#define TQC_BA_MAT_QUEUE_SIZE           10
#define TQC_BA_MAT_QUEUE_SLEEP_TIME_MAX 50
#define TQC_BA_MAT_QUEUE_SLEEP_TIME_MIN 5
#define TQC_BA_DISTANCE_SCALE           6  // for keypoits match filter.
#define TQC_BA_MAX_NORM                 0.3

// Debug Output string
#define TQC_DEBUG_STR_GLOBAL_T    "Global t"
#define TQC_DEBUG_STR_GLOBAL_R    "Global r"
#define TQC_DEBUG_STR_TRANSLATION "Translation"
#define TQC_DEBUG_STR_ROTATION    "Rotation"
#define TQC_DEBUG_STR_MATRIX      "Pose"
#endif // __TQC_DEFINES_H
