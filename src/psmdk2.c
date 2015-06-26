#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>
#include <conio.h>
#include "psmove.h"
#include "psmove_tracker.h"
#include "OVR_CAPI.h"
#include "OVR_CAPI_Keys.h"

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

    // Print the initial camera pose
    printDK2CameraPose(HMD);

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