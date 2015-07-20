#include <stdio.h>  // printf
#include <stdlib.h> // calloc, free
#include <assert.h>
#include <Eigen/Dense>
#include "psmove.h"
#include "psmove_tracker.h"
#include "OVR_CAPI.h"
#include "Extras/OVR_Math.h"

#define NPOSES 300

OVR::Matrix4f getDK2CameraInv44(ovrHmd HMD) {

    ovrTrackingState dk2state;
    dk2state = ovrHmd_GetTrackingState(HMD, 0.0);
    OVR::Posef campose(dk2state.CameraPose);
    campose.Rotation.Normalize();  // Probably does nothing as the SDK returns normalized quats anyway.
    campose.Translation *= 100.0;  // m -> cm
    
    // Print to file - for testing in Matlab
    char *fpath = psmove_util_get_file_path("output_camerapose.csv");
    FILE *fp = fopen(fpath, "w");
    free(fpath);
    fprintf(fp, "%f, %f, %f, %f, %f, %f, %f\n",
        campose.Translation.x, campose.Translation.y, campose.Translation.z,
        campose.Rotation.w, campose.Rotation.x, campose.Rotation.y, campose.Rotation.z);
    fclose(fp);

    OVR::Matrix4f camMat(campose);
    
    printf("Camera pose 4x4:\n");
    printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
        camMat.M[0][0], camMat.M[0][1], camMat.M[0][2], camMat.M[0][3],
        camMat.M[1][0], camMat.M[1][1], camMat.M[1][2], camMat.M[1][3],
        camMat.M[2][0], camMat.M[2][1], camMat.M[2][2], camMat.M[2][3],
        camMat.M[3][0], camMat.M[3][1], camMat.M[3][2], camMat.M[3][3]);

    camMat.InvertHomogeneousTransform();
    printf("Inverted camera pose 4x4:\n");
    printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
        camMat.M[0][0], camMat.M[0][1], camMat.M[0][2], camMat.M[0][3],
        camMat.M[1][0], camMat.M[1][1], camMat.M[1][2], camMat.M[1][3],
        camMat.M[2][0], camMat.M[2][1], camMat.M[2][2], camMat.M[2][3],
        camMat.M[3][0], camMat.M[3][1], camMat.M[3][2], camMat.M[3][3]);

    return camMat;
}

void ovrmat2eigmat(OVR::Matrix4f& in_ovr, Eigen::Matrix4f& in_eig)
{
    // The following can probably be done with a simple memcpy but oh well.
    int row, col;
    for (row = 0; row < 4; row++)
    {
        for (col = 0; col < 4; col++)
        {
            in_eig(row, col) = in_ovr.M[row][col];
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
    int isTracked = 0;
    int i = 0;
    controllers[i] = psmove_connect_by_id(i);

    while (1) {
        if (i == 0 && arg >= 3)
        {
            result = psmove_tracker_enable_with_color(tracker, controllers[i],
                atoi(args[1]), atoi(args[2]), atoi(args[3]));
            printf("Setting LEDS for controller 1 from command-line r: %i, g: %i, b: %i\n",
                atoi(args[1]), atoi(args[2]), atoi(args[3]));
        }
        else {
            result = psmove_tracker_enable(tracker, controllers[i]);
        }
        if (result == Tracker_CALIBRATED) {
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
#if defined(OVR_OS_WIN32)
    ovrHmd_Create(0, &HMD);
#elif defined(OVR_OS_MAC)
    HMD = ovrHmd_Create(0);
#endif
    ovrresult = ovrHmd_ConfigureTracking(HMD,
                ovrTrackingCap_Orientation |
                ovrTrackingCap_MagYawCorrection |
                ovrTrackingCap_Position, 0);

    // Initialize variables for our loop.
    OVR::Posef dk2pose;               // The DK2 pose
    OVR::Matrix4f dk2mat;             // The DK2 HMD pose in 4x4
    OVR::Posef psmovepose;            // The psmove pose
    OVR::Matrix4f psmovemat;          // The PSMove pose in 4x4
    OVR::Matrix4f camera_invxform;    // The DK2 camera pose inverse in 4x4

    psmovepose.Rotation = OVR::Quatf::Identity();  // PSMove orientation not used by this algorithm.
    
    int p = 0;                          // NPOSES counter
    Eigen::MatrixXf A(NPOSES * 3, 15);  // X = A/b
    Eigen::VectorXf b(NPOSES * 3);
    Eigen::Matrix4f dk2eig;             // DK2 pose in Eigen 4x4 mat
    Eigen::Matrix3f RMi;                // Transpose of inner 3x3 of DK2 pose
    
    // Start with current camera pose inverse
    camera_invxform = getDK2CameraInv44(HMD);

    // Print the column headers
    char *output_fpath = psmove_util_get_file_path("output.txt");
    FILE *output_fp = fopen(output_fpath, "w");
    free(output_fpath);
    fprintf(output_fp, "psm_px,psm_py,psm_pz,psm_ow,psm_ox,psm_oy,psm_oz,dk2_px,dk2_py,dk2_pz,dk2_ow,dk2_ox,dk2_oy,dk2_oz\n");
    printf("Hold the PSMove controller firmly against the DK2.\n");
    printf("Move them together through the workspace and press the Move button to sample (%d samples required).\n", NPOSES);
    fflush(stdout);
    while (p < NPOSES)
    {
        // Get PSMove position
        psmove_tracker_update_image(tracker);               // Refresh camera
        psmove_tracker_update_cbb(tracker, NULL);           // Update position based on refreshed image
        isTracked = psmove_tracker_get_status(tracker, controllers[i]) == Tracker_TRACKING;
        psmove_tracker_get_location(tracker, controllers[i],// Copy location to psmovepose
            &psmovepose.Translation.x, &psmovepose.Translation.y, &psmovepose.Translation.z);

        // Get PSMove buttons
        while (psmove_poll(controllers[i]));
        buttons = psmove_get_buttons(controllers[i]);
        
        // If circle button is pressed on PSMove then recenter HMD
        if (buttons & Btn_CIRCLE)
        {
            ovrHmd_RecenterPose(HMD);
            camera_invxform = getDK2CameraInv44(HMD);
        }

        // Get DK2 tracking state (contains pose)
        dk2state = ovrHmd_GetTrackingState(HMD, 0.0);
        dk2pose = dk2state.HeadPose.ThePose;
        dk2pose.Rotation.Normalize();
        dk2pose.Translation *= 100.0;
        
        // If MOVE button is pressed on PSMove, sample the position
        if (buttons & Btn_MOVE && isTracked)
        {
            fprintf(output_fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                psmovepose.Translation.x, psmovepose.Translation.y, psmovepose.Translation.z,
                psmovepose.Rotation.w, psmovepose.Rotation.x, psmovepose.Rotation.y, psmovepose.Rotation.z,
                dk2pose.Translation.x, dk2pose.Translation.y, dk2pose.Translation.z,
                dk2pose.Rotation.w, dk2pose.Rotation.x, dk2pose.Rotation.y, dk2pose.Rotation.z);

            dk2mat = OVR::Matrix4f(dk2pose);
            dk2mat = camera_invxform * dk2mat;
            psmovemat = OVR::Matrix4f(psmovepose);

            /*
            if (p == 0)
            {
                printf("PSMove pose V7:\n");
                printf("%f, %f, %f, %f, %f, %f, %f\n",
                    psmovepose.Translation.x, psmovepose.Translation.y, psmovepose.Translation.z,
                    psmovepose.Rotation.w, psmovepose.Rotation.x, psmovepose.Rotation.y, psmovepose.Rotation.z);
                printf("PSMove pose 4x4:\n");
                printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
                    psmovemat.M[0][0], psmovemat.M[0][1], psmovemat.M[0][2], psmovemat.M[0][3],
                    psmovemat.M[1][0], psmovemat.M[1][1], psmovemat.M[1][2], psmovemat.M[1][3],
                    psmovemat.M[2][0], psmovemat.M[2][1], psmovemat.M[2][2], psmovemat.M[2][3],
                    psmovemat.M[3][0], psmovemat.M[3][1], psmovemat.M[3][2], psmovemat.M[3][3]);
            }
            */

            ovrmat2eigmat(dk2mat, dk2eig);
            RMi = dk2eig.topLeftCorner(3, 3).transpose();           // inner 33 transposed

            /*
            int i, j;
            for (i = 0; i < 4; i++)
            {
                printf("\n");
                for (j = 0; j < 4; j++)
                {
                    printf("%5.2f,", psmovemat(i, j));
                }
                printf("\t\t");
                for (j = 0; j < 4; j++)
                {
                    printf("%5.2f,", dk2mat(i, j));
                }
            }
            */

            A.block<3, 3>(p * 3, 0) = RMi * psmovemat.M[0][3];
            A.block<3, 3>(p * 3, 3) = RMi * psmovemat.M[1][3];
            A.block<3, 3>(p * 3, 6) = RMi * psmovemat.M[2][3];
            A.block<3, 3>(p * 3, 9) = RMi;
            A.block<3, 3>(p * 3, 12) = -Eigen::Matrix3f::Identity();
            b.segment(p * 3, 3) = RMi * dk2eig.block<3, 1>(0, 3);
            p++;

            printf("\rSampled %d / %d poses.", p, NPOSES);
            fflush(stdout);
        }

        if (buttons & Btn_SELECT)
            break;
    }

    if (p == NPOSES)
    {
        // TODO: Remove outliers

        /*
        for (p = 0; p < NPOSES * 3; p++)
        {
            printf("%4d: %5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\t%5.2f\n", p,
                A(p, 0), A(p, 1), A(p, 2), A(p, 3), A(p, 4),
                A(p, 5), A(p, 6), A(p, 7), A(p, 8), A(p, 9),
                A(p, 10), A(p, 11), A(p, 12), A(p, 13), A(p, 14), b(p));
        }
        */
        Eigen::VectorXf x(15);
        x = A.colPivHouseholderQr().solve(b);
        //globalxfm = reshape(x(1:12), 3, 4);
        //localxfm = [1 0 0 x(12); 0 1 0 x(13); 0 0 1 x(14); 0 0 0 1];
        printf("\nglobalxfm:\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
            x(0), x(3), x(6), x(9),
            x(1), x(4), x(7), x(10),
            x(2), x(5), x(8), x(11));
        printf("\nlocalxfm:\n%f,%f,%f\n", x(12), x(13), x(14));

        // Save XML to home directory
        char *fpath = psmove_util_get_file_path("transform.csv");
        FILE *fp = fopen(fpath, "w");
        free(fpath);

        // Print XML
        /*
        fprintf(fp, "< ? xml version = \"1.0\" encoding = \"UTF - 8\" ? >\n");
        fprintf(fp, "<globalxform>\n");
        int i, j;
        for (i = 0; i < 3; i++)
        {
            for (j = 0; j < 4; j++)
            {
                fprintf(fp, "    <value row=%d column=%d>%f</value>\n", i, j, x(j * 3 + i));
            }
        }
        fprintf(fp, "</globalxform>\n");
        */

        // Print simple csv
        fprintf(fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",
            x(0), x(1), x(2), x(3), x(4), x(5), x(6), x(7), x(8), x(9), x(10), x(11));

        fclose(fp);

    }
    psmove_disconnect(controllers[i]);
    psmove_tracker_free(tracker);
    free(controllers);

    ovrHmd_Destroy(HMD);
    ovr_Shutdown();

    return 0;
}