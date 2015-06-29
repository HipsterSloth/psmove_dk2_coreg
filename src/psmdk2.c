#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>
#include <conio.h>
#include "psmove.h"
#include "psmove_tracker.h"
#include "OVR_CAPI.h"
#include "OVR_CAPI_Keys.h"

#define NPOSES 300

void printDK2CameraPose(ovrHmd HMD) {
    ovrTrackingState dk2state;
    dk2state = ovrHmd_GetTrackingState(HMD, 0.0);

    printf("New DK2 camera leveled pose: %f, %f, %f, %f, %f, %f, %f\n",
        100.0*dk2state.LeveledCameraPose.Position.x,
        100.0*dk2state.LeveledCameraPose.Position.y,
        100.0*dk2state.LeveledCameraPose.Position.z,
        dk2state.LeveledCameraPose.Orientation.x,
        dk2state.LeveledCameraPose.Orientation.y,
        dk2state.LeveledCameraPose.Orientation.z,
        dk2state.LeveledCameraPose.Orientation.w);
}

ovrMatrix4f invertMat44(ovrMatrix4f in){

    ovrMatrix4f out;

    // Transpose inner 3x3
    out.M[0][0] = in.M[0][0];
    out.M[0][1] = in.M[1][0];
    out.M[0][2] = in.M[2][0];
    out.M[1][0] = in.M[0][1];
    out.M[1][1] = in.M[1][1];
    out.M[1][2] = in.M[2][1];
    out.M[2][0] = in.M[0][2];
    out.M[2][1] = in.M[1][2];
    out.M[2][2] = in.M[2][2];

    // Right column.
    out.M[0][3] = -out.M[0][0] * in.M[0][3] - out.M[0][1] * in.M[1][3] - out.M[0][2] * in.M[2][3];
    out.M[1][3] = -out.M[1][0] * in.M[0][3] - out.M[1][1] * in.M[1][3] - out.M[1][2] * in.M[2][3];
    out.M[2][3] = -out.M[2][0] * in.M[0][3] - out.M[2][1] * in.M[1][3] - out.M[2][2] * in.M[2][3];

    // Bottom row
    out.M[3][0] = 0;
    out.M[3][1] = 0;
    out.M[3][2] = 0;
    out.M[3][3] = 1;

    return out;
}

//http://www.euclideanspace.com/maths/geometry/affine/conversions/quaternionToMatrix/
void pose2mat(ovrPosef camPose, ovrMatrix4f* out, float scale) {
    ovrVector3f p = camPose.Position;
    p.x = p.x * scale;
    p.y = p.y * scale;
    p.z = p.z * scale;
    ovrQuatf q = camPose.Orientation;
    float sqw = q.w*q.w;
    float sqx = q.x*q.x;
    float sqy = q.y*q.y;
    float sqz = q.z*q.z;
    out->M[0][0] = sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz =1
    out->M[1][1] = -sqx + sqy - sqz + sqw;
    out->M[2][2] = -sqx - sqy + sqz + sqw;
    float tmp1 = q.x*q.y;
    float tmp2 = q.z*q.w;
    out->M[0][1] = 2.0 * (tmp1 + tmp2);
    out->M[1][0] = 2.0 * (tmp1 - tmp2);
    tmp1 = q.x*q.z;
    tmp2 = q.y*q.w;
    out->M[0][2] = 2.0 * (tmp1 - tmp2);
    out->M[2][0] = 2.0 * (tmp1 + tmp2);
    tmp1 = q.y*q.z;
    tmp2 = q.x*q.w;
    out->M[1][2] = 2.0 * (tmp1 + tmp2);
    out->M[2][1] = 2.0 * (tmp1 - tmp2);
    out->M[0][3] = p.x - p.x * out->M[0][0] - p.y * out->M[0][1] - p.z * out->M[0][2];
    out->M[1][3] = p.y - p.x * out->M[1][0] - p.y * out->M[1][1] - p.z * out->M[1][2];
    out->M[2][3] = p.z - p.x * out->M[2][0] - p.y * out->M[2][1] - p.z * out->M[2][2];
    out->M[3][0] = out->M[3][1] = out->M [3][2] = 0.0;
    out->M[3][3] = 1.0;
}

ovrMatrix4f mat44xmat44(ovrMatrix4f lh, ovrMatrix4f rh)
{
    ovrMatrix4f out;
    int i, j;
    for (i = 0; i < 4; i++){
        for (j = 0; j < 4; j++)
        {
            out.M[i][j] = lh.M[i][0] * rh.M[0][j] + lh.M[i][1] * rh.M[1][j] + lh.M[i][2] * rh.M[2][j] + lh.M[i][3] * rh.M[3][j];
        }
    }
}

int main(int arg, char** args) {

    // Setup PSMove
    int count = psmove_count_connected();
    PSMove **controllers = (PSMove **)calloc(count, sizeof(PSMove *));
    PSMoveTracker* tracker = psmove_tracker_new();
    psmove_tracker_set_mirror(tracker, PSMove_True);
    
    int result;
    int i = 0;
    controllers[i] = psmove_connect_by_id(i);
    while (1) {
        result = psmove_tracker_enable(tracker, controllers[i]);
        if (result == Tracker_CALIBRATED) {
            enum PSMove_Bool auto_update_leds =
                psmove_tracker_get_auto_update_leds(tracker,
                        controllers[i]);
            break;
        } else {
            printf("ERROR - retrying\n");
        }
    }
    assert(psmove_has_calibration(controllers[i]));
    psmove_enable_orientation(controllers[i], PSMove_True);
    assert(psmove_has_orientation(controllers[i]));
    int buttons = psmove_get_buttons(controllers[i]);
    
    // Setup DK2
    ovrBool ovrresult;
    ovrHmd HMD;
    ovrTrackingState dk2state;
    ovrresult = ovr_Initialize(0);
    ovrHmd_Create(0, &HMD);
    ovrresult = ovrHmd_ConfigureTracking(HMD,
                ovrTrackingCap_Orientation |
                ovrTrackingCap_MagYawCorrection |
                ovrTrackingCap_Position, 0);

    // Get the camera pose inverse
    dk2state = ovrHmd_GetTrackingState(HMD, 0.0);
    ovrMatrix4f dk2caminv;
    pose2mat(dk2state.LeveledCameraPose, &dk2caminv, 100.0);
    dk2caminv = invertMat44(dk2caminv);

    // Print the initial camera pose
    printDK2CameraPose(HMD);

    ovrMatrix4f dk2mat;
    ovrPosef psmovepose;
    psmovepose.Orientation.w = 1;
    psmovepose.Orientation.x = 0;
    psmovepose.Orientation.y = 0;
    psmovepose.Orientation.z = 0;
    ovrMatrix4f psmovemat;

    int p = 0; //NPOSES
    int j, k;
    float A[NPOSES*3][15];
    float b[NPOSES*3];

    // Print the data
    printf("psm_px,psm_py,psm_pz,psm_ox,psm_oy,psm_oz,psm_ow,dk2_px,dk2_py,dk2_pz,dk2_ox,dk2_oy,dk2_oz,dk2_ow\n");
    while (1)
    {
        psmove_tracker_update_image(tracker);
        psmove_tracker_update(tracker, NULL);

        // Get PSMove position
        float psm_px, psm_py, psm_pz;
        psmove_tracker_get_location(tracker, controllers[i], &psm_px, &psm_py, &psm_pz);
        
        // Get PSMove orientation
        while (psmove_poll(controllers[i]));
        float psm_ox, psm_oy, psm_oz, psm_ow;
        psmove_get_orientation(controllers[i], &psm_ow, &psm_ox, &psm_oy, &psm_oz);


        // Get PSMove buttons
        buttons = psmove_get_buttons(controllers[i]);
        
        if (buttons & Btn_CIRCLE)
        {
            ovrHmd_RecenterPose(HMD);
            printDK2CameraPose(HMD); //Whenever the Hmd pose is recentered, the camera position and yaw changes.

            //psmove_reset_orientation(controllers[i]);
            //psmove_tracker_reset_location(tracker, controllers[i]);
        }

        // Get DK2 tracking state (contains pose)
        dk2state = ovrHmd_GetTrackingState(HMD, 0.0);

        if (buttons & Btn_MOVE)
        {
            printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                psm_px, psm_py, psm_pz,
                psm_ox, psm_oy, psm_oz, psm_ow,
                100.0*dk2state.HeadPose.ThePose.Position.x,
                100.0*dk2state.HeadPose.ThePose.Position.y,
                100.0*dk2state.HeadPose.ThePose.Position.z,
                dk2state.HeadPose.ThePose.Orientation.x,
                dk2state.HeadPose.ThePose.Orientation.y,
                dk2state.HeadPose.ThePose.Orientation.z,
                dk2state.HeadPose.ThePose.Orientation.w);

            pose2mat(dk2state.LeveledCameraPose, &dk2mat, 100.0);
            dk2mat = mat44xmat44(dk2caminv, dk2mat);

            psmovepose.Position.x = psm_px;
            psmovepose.Position.y = psm_py;
            psmovepose.Position.z = psm_pz;
            pose2mat(psmovepose, &psmovemat, 1.0);

            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    A[p * 3 + i][3 * k + j] = dk2mat.M[j][i] * psmovemat.M[k][3];
                }
                b[p * 3 + i] = dk2mat.M[i][0] * dk2mat.M[i][0] + dk2mat.M[i][1] * dk2mat.M[i][1] + dk2mat.M[i][2] * dk2mat.M[i][2];
            }
        }

        if (buttons & Btn_SELECT)
            break;
    }
    
    psmove_disconnect(controllers[i]);
    psmove_tracker_free(tracker);
    free(controllers);
    
    ovrHmd_Destroy(HMD);
    ovr_Shutdown();
    
    return 0;
}