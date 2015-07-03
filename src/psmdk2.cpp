#include <stdio.h>  // printf
#include <stdlib.h> // calloc, free
#include <assert.h>
#include <Eigen/Dense>
#include "psmove.h"
#include "psmove_tracker.h"
#include "OVR_CAPI.h"

#define NPOSES 300

Eigen::Matrix3f vecKroneckerProduct(Eigen::Vector3f lh, Eigen::Vector3f rh)
{
    Eigen::Matrix3f out;
    int i, j;
    for (i = 0; i < 3; i++)
    {
        for (j = 0; j < 3; j++)
        {
            out(i, j) = lh(i)*rh(j);
        }
    }
    return out;
}

Eigen::Matrix4f invertMat44(Eigen::Matrix4f in){
    // Invert a 4x4 affine transformation matrix.
    // Some tricks can be used for this specific type of matrix.

    Eigen::Matrix4f out;

    // Transpose inner 3x3
    out.topLeftCorner(3, 3) = in.topLeftCorner(3, 3).transpose();

    // Right column.
    out.block<3, 1>(0, 3) = (-1 * in.topLeftCorner(3, 3).transpose()) * in.block<3, 1>(0, 3);

    // Bottom row
    out.row(3) << 0, 0, 0, 1;

    return out;
}

float fixup(float val)
{
    float out;
    if (abs(val) < 0.0000001)
    {
        out = 0;
    }
    else if(abs(val - 1) < 0.0000001)
    {
        out = 1;
    }
    else if (abs(val + 1) < 0.0000001)
    {
        out = -1;
    }
    else
    {
        out = val;
    }
    return out;
}

Eigen::Matrix3f skewSymm(Eigen::Vector3f in){
    Eigen::Matrix3f out;
    out <<
         0,     -in(2), in(1),
         in(2),  0,    -in(0),
        -in(1),  in(0), 0;
    return out;
}

Eigen::Matrix4f pose2mat(ovrPosef pose, float scale) {
    Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    ovrVector3f p = pose.Position;
    ovrQuatf q = pose.Orientation;

    // Scale is used only to convert units (e.g. m from DK2 to cm for PSMove)
    p.x = p.x * scale;
    p.y = p.y * scale;
    p.z = p.z * scale;

    Eigen::Vector3f u;
    u << q.x, q.y, q.z;
    u /= u.norm();

    float c = fixup(cos(q.w));
    float s = fixup(sin(q.w));

    rotate.topLeftCorner(3, 3) = c*Eigen::Matrix3f::Identity() +
        (1 - c)*vecKroneckerProduct(u, u.transpose()) +
        s*skewSymm(u);

    translate.block<3, 1>(0, 3) << p.x, p.y, p.z;

    return translate * rotate;
}

Eigen::Matrix4f getDK2CameraInv44(ovrHmd HMD) {

    ovrTrackingState dk2state;
    dk2state = ovrHmd_GetTrackingState(HMD, 0.0);

    // Print to file - for testing in Matlab
    char *fpath = psmove_util_get_file_path("output_camerapose.csv");
    FILE *fp = fopen(fpath, "w");
    free(fpath);
    fprintf(fp, "%f, %f, %f, %f, %f, %f, %f\n",
        100.0*dk2state.LeveledCameraPose.Position.x,
        100.0*dk2state.LeveledCameraPose.Position.y,
        100.0*dk2state.LeveledCameraPose.Position.z,
        dk2state.LeveledCameraPose.Orientation.x,
        dk2state.LeveledCameraPose.Orientation.y,
        dk2state.LeveledCameraPose.Orientation.z,
        dk2state.LeveledCameraPose.Orientation.w);
    fclose(fp);

    Eigen::Matrix4f dk2caminv;
    dk2caminv = pose2mat(dk2state.LeveledCameraPose, 100.0);
    /*
    printf("Camera pose 4x4:\n");
    printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
        dk2caminv(0, 0), dk2caminv(0, 1), dk2caminv(0, 2), dk2caminv(0, 3),
        dk2caminv(1, 0), dk2caminv(1, 1), dk2caminv(1, 2), dk2caminv(1, 3),
        dk2caminv(2, 0), dk2caminv(2, 1), dk2caminv(2, 2), dk2caminv(2, 3),
        dk2caminv(3, 0), dk2caminv(3, 1), dk2caminv(3, 2), dk2caminv(3, 3));
    */
    dk2caminv = invertMat44(dk2caminv);
    /*
    printf("Inverted camera pose 4x4:\n");
    printf("%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n%f, %f, %f, %f\n",
        dk2caminv(0, 0), dk2caminv(0, 1), dk2caminv(0, 2), dk2caminv(0, 3),
        dk2caminv(1, 0), dk2caminv(1, 1), dk2caminv(1, 2), dk2caminv(1, 3),
        dk2caminv(2, 0), dk2caminv(2, 1), dk2caminv(2, 2), dk2caminv(2, 3),
        dk2caminv(3, 0), dk2caminv(3, 1), dk2caminv(3, 2), dk2caminv(3, 3));
    */
    return dk2caminv;
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
    Eigen::Matrix4f dk2mat;             // The DK2 HMD pose in 4x4
    Eigen::Matrix4f camera_invxform;    // The DK2 camera pose inverse in 4x4
    Eigen::Matrix4f psmovemat;          // The PSMove pose in 4x4
    ovrPosef psmovepose;                // The psmove pose in ovrPosef (easy 4x4 conversion)
    psmovepose.Orientation.w = 1;       // PSMove orientation not used by this algorithm.
    psmovepose.Orientation.x = 0;
    psmovepose.Orientation.y = 0;
    psmovepose.Orientation.z = 0;
    int p = 0;                          // NPOSES counter
    Eigen::MatrixXf A(NPOSES * 3, 15);  // X = A/b
    Eigen::VectorXf b(NPOSES * 3);
    Eigen::Matrix3f RMi;                // Transpose of inner 3x3 of DK2 pose
    
    // Start with current camera pose inverse
    camera_invxform = getDK2CameraInv44(HMD);

    // Print the column headers
    char *output_fpath = psmove_util_get_file_path("output.txt");
    FILE *output_fp = fopen(output_fpath, "w");
    free(output_fpath);
    fprintf(output_fp, "psm_px,psm_py,psm_pz,psm_ox,psm_oy,psm_oz,psm_ow,dk2_px,dk2_py,dk2_pz,dk2_ox,dk2_oy,dk2_oz,dk2_ow\n");
    printf("Hold the PSMove controller firmly against the DK2.\n");
    printf("Move them together through the workspace and press the Move button to sample (%d samples required).\n", NPOSES);
    fflush(stdout);
    while (p < NPOSES)
    {
        // Get PSMove position
        psmove_tracker_update_image(tracker);               // Refresh camera
        psmove_tracker_update(tracker, NULL);               // Update position based on refreshed image
        psmove_tracker_get_location(tracker, controllers[i],// Copy location to psmovepose
            &psmovepose.Position.x, &psmovepose.Position.y, &psmovepose.Position.z);

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

        // If MOVE button is pressed on PSMove, sample the position
        if (buttons & Btn_MOVE)
        {
            fprintf(output_fp, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
                psmovepose.Position.x, psmovepose.Position.y, psmovepose.Position.z,
                0.0, 0.0, 0.0, 1.0,
                100.0*dk2state.HeadPose.ThePose.Position.x,
                100.0*dk2state.HeadPose.ThePose.Position.y,
                100.0*dk2state.HeadPose.ThePose.Position.z,
                dk2state.HeadPose.ThePose.Orientation.x,
                dk2state.HeadPose.ThePose.Orientation.y,
                dk2state.HeadPose.ThePose.Orientation.z,
                dk2state.HeadPose.ThePose.Orientation.w);

            dk2mat = pose2mat(dk2state.HeadPose.ThePose, 100.0);    // dk2 hmd pose in 44 mat
            dk2mat = camera_invxform * dk2mat;                      // dk2 hmd pose 44 in dk2 camera space
            RMi = dk2mat.topLeftCorner(3, 3).transpose();           // inner 33 transposed
            psmovemat = pose2mat(psmovepose, 1.0);                  // psmove position in 44 mat

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

            A.block<3, 3>(p * 3, 0) = RMi * psmovemat(0, 3);
            A.block<3, 3>(p * 3, 3) = RMi * psmovemat(1, 3);
            A.block<3, 3>(p * 3, 6) = RMi * psmovemat(2, 3);
            A.block<3, 3>(p * 3, 9) = RMi;
            A.block<3, 3>(p * 3, 12) = -Eigen::Matrix3f::Identity();
            b.segment(p * 3, 3) = RMi * dk2mat.block<3, 1>(0, 3);
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
        printf("\n%f,%f,%f,%f\n%f,%f,%f,%f\n%f,%f,%f,%f\n",
            x(0), x(3), x(6), x(9),
            x(1), x(4), x(7), x(10),
            x(2), x(5), x(8), x(11));

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