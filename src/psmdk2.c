#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>
#include <conio.h>
#include "psmove.h"
#include "psmove_tracker.h"
#include "OVR_CAPI.h"
#include "OVR_CAPI_Keys.h"

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
    
    // TODO: Setup DK2
    ovrBool ovrresult;
    ovrresult = ovr_Initialize(0);
    ovrHmd HMD = ovrHmd_Create(0);
    ovrresult = ovrHmd_ConfigureTracking(HMD,
                ovrTrackingCap_Orientation |
                ovrTrackingCap_MagYawCorrection |
                ovrTrackingCap_Position, 0);
    ovrTrackingState dk2state;
    
    float psm_x_offset = 0.0;
    float psm_y_offset = 0.0;
    float psm_z_offset = 0.0;
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

        // Update DK2 state
        dk2state = ovrHmd_GetTrackingState(HMD, 0.0);
        
        buttons = psmove_get_buttons(controllers[i]);

        if (buttons & Btn_CIRCLE)
        {
            psm_x_offset = psm_px;
            psm_y_offset = psm_py;
            psm_z_offset = psm_pz;
            ovrHmd_RecenterPose(HMD);
            psmove_reset_orientation(controllers[i]);
        }

        if (buttons & Btn_MOVE)
            printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                psm_px-psm_x_offset, psm_py-psm_y_offset, psm_pz-psm_z_offset,
                psm_ox, psm_oy, psm_oz, psm_ow,
                dk2state.HeadPose.ThePose.Position.x,
                dk2state.HeadPose.ThePose.Position.y,
                dk2state.HeadPose.ThePose.Position.z,
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