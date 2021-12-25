#include <chrono>

#include <iostream>
#include <vector>
#include <algorithm>
#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#include "traj.h"
#include "draw.h"
#include "delfem2/rig_geo3.h"
#include "delfem2/glfw/viewer3.h"
#include "delfem2/glfw/util.h"
#include "delfem2/opengl/old/funcs.h"
#include "delfem2/opengl/old/rigv3.h"
#include "delfem2/rig_bvh.h"

namespace dfm2 = delfem2;

int Pose_Match_Best(std::vector<double>& pose_match_date, std::vector<double>& desired_future_traj, int database_size, int num_sample)
{
    double cost = 9999999.0;
    int target_frame = 0;
    int total_frame = database_size / num_sample;
    //std::cout << "total" << total_frame << std::endl;
    for (unsigned int iframe = 0; iframe < total_frame; iframe++)
    {
        double tempt_cost = 0.0f;
        for (unsigned int isample = 0; isample < 10; isample++)
        {
            tempt_cost += abs(pose_match_date[iframe * num_sample + isample] - desired_future_traj[isample]);
        }
        if (tempt_cost < cost)
        {
            target_frame = iframe;
            cost = tempt_cost;
        }
    }
    return target_frame;
}


void Gene_Pose_Match_Dataset(std::vector<double>& pose_match_date,const std::vector<double>* timeseries_date, int channelSize, int sample_frame)
{
    int size_date = timeseries_date->size();
    int size_timeseries = size_date / channelSize;
    for (unsigned int ich = 0; ich < size_timeseries; ich += 1)
    {
        for (unsigned int j = 1; j < 6; j += 1)
        {
            //const double val = (*timeseries_date)[channelSize * ich + j * sample_frame + 0];
            double x_root = (*timeseries_date)[channelSize * ich + 0];
            double z_root = (*timeseries_date)[channelSize * ich + 2];
            int ix = channelSize * ich + j * sample_frame * channelSize + 0;
            int iz = channelSize * ich + j * sample_frame * channelSize + 2;
            if (ix >= size_date)
            {
                pose_match_date.push_back(0.0f);
                pose_match_date.push_back(0.0f);
            }
            else 
            {
                double x_pos = (*timeseries_date)[ix] - x_root;
                double y_pos = (*timeseries_date)[iz] - z_root;
                pose_match_date.push_back(x_pos);
                pose_match_date.push_back(y_pos);
            }
        }
    }
}

void SetPose_BioVisionHierarchy_Rotate(
    std::vector<dfm2::CRigBone>& bones,
    const std::vector<dfm2::CChannel_BioVisionHierarchy>& channels,
    const double* values,
    dfm2::CVec2d& traj) {
    for (auto& bone : bones) {
        bone.quatRelativeRot[0] = 0.0;
        bone.quatRelativeRot[1] = 0.0;
        bone.quatRelativeRot[2] = 0.0;
        bone.quatRelativeRot[3] = 1.0;
    }
    const size_t nch = channels.size();
    int traj_index = 0;
    for (unsigned int ich = 0; ich < nch; ++ich) {
        const int ibone = channels[ich].ibone;
        const int iaxis = channels[ich].iaxis;
        const bool isrot = channels[ich].isrot;
        const double val = values[ich];
        assert(ibone < (int)bones.size());
        assert(iaxis >= 0 && iaxis < 3);
        if (!isrot) {
            if (traj_index == 0)
                bones[ibone].transRelative[iaxis] = traj.x;
            else if (traj_index == 2)
                bones[ibone].transRelative[iaxis] = traj.y;
            else
                bones[ibone].transRelative[iaxis] = val;
            traj_index += 1;
        }
        else {
            const double ar = val * M_PI / 180.0;
            double v0[3] = { 0, 0, 0 };
            v0[iaxis] = 1.0;
            double dq[4] = { v0[0] * sin(ar * 0.5), v0[1] * sin(ar * 0.5), v0[2] * sin(ar * 0.5), cos(ar * 0.5) };
            double qtmp[4];
            dfm2::QuatQuat(qtmp,
                bones[ibone].quatRelativeRot, dq);
            dfm2::Copy_Quat(bones[ibone].quatRelativeRot, qtmp);
        }
    }
    UpdateBoneRotTrans(bones);
}

void joystick_callback(int jid, int event)
{
    if (event == GLFW_CONNECTED)
    {
        std::cout << "Controller Connected" << std::endl;
        // The joystick was connected
    }
    else if (event == GLFW_DISCONNECTED)
    {
        std::cout << "Controller Disconnected" << std::endl;
        std::cout << "Recommend connecting a controller for play" << std::endl;

        // The joystick was disconnected
    }
}

static void error_callback(int error, const char* description) {
    fputs(description, stderr);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}


float x_target_v = 0.f;
float y_target_v = 0.f;
float keyboard_speed_mag = 25.0f;


int main(int argc, char* argv[]) {
    const int sample_frame = 20;
    
    //const int boneSize= 38;
    /*
    std::vector<dfm2::CRigBone> readBone;
    readBone.resize(38);
    std::ifstream inFile;
    inFile.open(std::string(PATH_SOURCE_DIR) + "/../data/motion_match_data_1.bin", std::ios::in | std::ios::binary);

    inFile.read((char*)&readBone, readBone.size());
    if (inFile.fail()) { std::cout << "Load Binary Error" << std::endl; return false; }
    std::cout << "size of current bone" << std::endl;
    std::cout << readBone.size() << std::endl;
    */
   
    

    std::vector<std::string> vec_path_bvh = {
        "/../data/LocomotionFlat03_000_mirror.bvh",
        "/../data/LocomotionFlat02_000_mirror.bvh",
        "/../data/LocomotionFlat01_000_mirror.bvh",
    };
    std::string ipath = "/../data/LocomotionFlat01_000.bvh";
    //for (auto& ipath : vec_path_bvh) {
        std::string path_bvh = std::string(PATH_SOURCE_DIR) + ipath;
        // read bvh 
        std::vector<dfm2::CRigBone> aBone;
        std::vector<dfm2::CChannel_BioVisionHierarchy> aChannelRotTransBone;
        size_t nframe = 0;
        double frame_time;
        std::string header_bvh;
        std::vector<double> vec_bvh_time_series_data;
        Read_BioVisionHierarchy(
            aBone,
            aChannelRotTransBone,
            nframe,
            frame_time,
            vec_bvh_time_series_data,
            header_bvh,
            path_bvh);
        dfm2::UpdateBoneRotTrans(aBone);
        std::cout << "nFame: " << nframe << std::endl;
        std::cout << "aBone: " << aBone.size() << std::endl;
        std::cout << "aChannels: " << aBone.size() << std::endl;
        assert(aBone.size() == 38 && aChannelRotTransBone.size() == 96);
        std::cout << "Data: " << vec_bvh_time_series_data.size()/96 << std::endl;

        std::vector<double> posematch_date;
        Gene_Pose_Match_Dataset(posematch_date, &vec_bvh_time_series_data, aChannelRotTransBone.size(), sample_frame);
        std::cout << posematch_date.size() << std::endl;

    //}

        //while (!fin.eof()) {
        //    double a = -1;
        //    fin >> a;
        //    if (!fin) { break; }  // the line break at the end of the file
        //    vec_phase.push_back(a);
        //}
        {
            std::ofstream outFile;
            std::string name = "motion_match_data_1";
            outFile.open(
                std::string(PATH_SOURCE_DIR) +
                "/../data/" + name +
                ".bin", std::ios::binary);
            outFile.write(
                reinterpret_cast<const char*>(aBone.data()),
                sizeof(double) * aBone.size());
            outFile.close();
        }
        //exit(3);
    class MytempViewer : public delfem2::glfw::CViewer3 {
    public:
        MytempViewer() : CViewer3(45) {
        }

        void key_press(int key, int mods) override {
            delfem2::glfw::CViewer3::key_press(key, mods);
            if (key == GLFW_KEY_F) {
                if (keyboard_speed_mag == 25.0f)
                    keyboard_speed_mag = 8.0f;
                else
                    keyboard_speed_mag = 25.0f;
            }
            if (key == GLFW_KEY_X) {
                x_target_v = 0.0f;
                y_target_v = 0.f;
            }
        }

        void key_repeat(int key, int mods) override {
            delfem2::glfw::CViewer3::key_press(key, mods);
            if (key == GLFW_KEY_W) {
                x_target_v = 0.f;
                y_target_v = -keyboard_speed_mag;
            }
            if (key == GLFW_KEY_S) {
                x_target_v = 0.f;
                y_target_v = keyboard_speed_mag;
            }
            if (key == GLFW_KEY_A) {
                x_target_v = -keyboard_speed_mag;
                y_target_v = 0.f;
            }
            if (key == GLFW_KEY_D) {
                x_target_v = keyboard_speed_mag;
                y_target_v = 0.f;
            }
        }
    };

    MytempViewer viewer_source;

    viewer_source.width = 1920;
    viewer_source.height = 1080;
    viewer_source.window_title = "Motion Matching";
    viewer_source.view_rotation = std::make_unique<delfem2::ModelView_Ytop>();

    delfem2::glfw::InitGLOld();

    viewer_source.OpenWindow();

    delfem2::opengl::setSomeLighting();

    std::cout << "run starts" << std::endl;

    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(EXIT_FAILURE);

    if (!viewer_source.window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    glfwMakeContextCurrent(viewer_source.window);

    //======
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(viewer_source.window, true);
    ImGui_ImplOpenGL2_Init();

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    //=======

    int path_count = -1;

    static float f = 0.01f;
    double speed_x = 0.0f;
    double speed_y = 0.0f;
    //damper_p the_p = damper_p();

    enum
    {
        TRAJ_MAX = 120,
        TRAJ_SUB = 20,
        PRED_MAX = 6,
        PRED_SUB = 20,
    };

    float trajx_prev[TRAJ_MAX];
    float trajy_prev[TRAJ_MAX];

    float predx[PRED_MAX], predy[PRED_MAX];
    float predxv[PRED_MAX], predyv[PRED_MAX];
    float predxa[PRED_MAX], predya[PRED_MAX];

    float halflife = 0.70f;
    float dt = 1.0 / 60.0f;
    //float timescale = 240.0f;
    float trajx = 0.0f;
    float trajy = 0.0f;
    float trajxv = 0.0, trajyv = 0.0;
    float trajxa = 0.0, trajya = 0.0;
    float traj_xv_goal = 0.0;
    float traj_yv_goal = 0.0;


    for (int i = 0; i < TRAJ_MAX; i++)
    {
        trajx_prev[i] = 0.0f;
        trajy_prev[i] = 0.0f;
    }

    float velocity_mag = 10.0f;
    dfm2::CVec2d face_dirZ(1.0f, 0.f);
    static int iframe = 0;
    const int nch = aChannelRotTransBone.size();
    while (!glfwWindowShouldClose(viewer_source.window))
    {
        glfwMakeContextCurrent(viewer_source.window);


        bool facing_control_mode = false;


        Floor floor{ 100, +0.1 };


        viewer_source.DrawBegin_oldGL();
        ::glEnable(GL_LINE_SMOOTH);
        ::glEnable(GL_BLEND);
        ::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);



        for (int i = TRAJ_MAX - 1; i > 0; i--)
        {
            trajx_prev[i] = trajx_prev[i - 1];
            trajy_prev[i] = trajy_prev[i - 1];
        }



        float gamepad_x = 0.0f;
        float gamepad_y = 0.0f;
        float gamepad2_x = 0.0f;
        float gamepad2_y = 0.0f;

        bool buttonX = false;
        bool buttonY = false;
        bool buttonA = false;
        bool buttonB = false;


        // input bind with controller ++++++++++++
        bool check_controller = false;
        int present = 0;
        glfwSetJoystickCallback(joystick_callback);
        if (glfwJoystickPresent(GLFW_JOYSTICK_1) == GLFW_TRUE)
        {

            present = glfwJoystickPresent(GLFW_JOYSTICK_1);

            if (present == 1)
            {
                check_controller = true;
                int axesCount;
                const float* axes = glfwGetJoystickAxes(GLFW_JOYSTICK_1, &axesCount);
                gamepad_x = axes[0];
                gamepad_y = axes[1];
                gamepad2_x = abs(axes[2]) < 0.05 ? 0.0f : axes[2] / 100;
                gamepad2_y = abs(axes[3]) < 0.05 ? 0.0f : axes[3] / 100;
                float gamepad_mag = sqrtf(gamepad_x * gamepad_x + gamepad_y * gamepad_y);

                int triggerRT = axes[5];
                int triggerLT = axes[4];
                //std::cout << "Right X : " << axes[2] << std::endl;
                //std::cout << "Right Y : " << axes[3] << std::endl;
                //std::cout << "trigger LT : " << axes[4] << std::endl;
                //std::cout << "trigger RT : " << axes[5] << std::endl;
                //std::cout << "trigger R3 : " << axes[6] << std::endl;
                //std::cout << "trigger R4 : " << axes[7] << std::endl;

                if (gamepad_mag > 0.2f)
                {
                    float gamepaddirx = gamepad_x / gamepad_mag;
                    float gamepaddiry = gamepad_y / gamepad_mag;
                    float gamepadclippedmag = gamepad_mag > 1.0f ? 1.0f : gamepad_mag * gamepad_mag;
                    gamepad_x = gamepaddirx * gamepadclippedmag;
                    gamepad_y = gamepaddiry * gamepadclippedmag;
                }
                else
                {
                    gamepad_x = 0.0f;
                    gamepad_y = 0.0f;
                }
                //22.0f close to run speed 
                //
                if (triggerLT == 1) {
                    facing_control_mode = true;
                }
                if (triggerRT == 1) {
                    // run

                    f = damper(f, 0.01f, 0.1f);
                    //f = 0.01f;
                    halflife = damper(halflife, 0.7f, 0.1f);
                    //halflife = 0.7f;
                    velocity_mag = damper(velocity_mag, 22.0f, 0.1f);
                    traj_xv_goal = gamepad_x * velocity_mag;
                    traj_yv_goal = gamepad_y * velocity_mag;
                }
                else
                {
                    // walk 

                    f = damper(f, 0.006f, 0.1f);
                    //f = 0.006f;
                    halflife = damper(halflife, 0.9, 0.1f);
                    //halflife = 0.4f;
                    velocity_mag = damper(velocity_mag, 10.0f, 0.1f);
                    traj_xv_goal = gamepad_x * velocity_mag;
                    traj_yv_goal = gamepad_y * velocity_mag;
                }
            }

            int buttonCont;
            const unsigned char* buttons = glfwGetJoystickButtons(GLFW_JOYSTICK_1, &buttonCont);
            //0 = A, 1 = B, 2 = X, 3 = Y
            if (GLFW_PRESS == buttons[0])
            {
                buttonA = true;
            }
            else if (GLFW_RELEASE == buttons[0])
            {
                buttonA = false;
            }
            if (GLFW_PRESS == buttons[1])
            {
                buttonB = true;
            }
            else if (GLFW_RELEASE == buttons[1])
            {
                buttonB = false;
            }
            if (GLFW_PRESS == buttons[2])
            {
                buttonX = true;
            }
            else if (GLFW_RELEASE == buttons[2])
            {
                buttonX = false;
            }
            if (GLFW_PRESS == buttons[3])
            {
                buttonY = true;
            }
            else if (GLFW_RELEASE == buttons[3])
            {
                buttonY = false;
            }
        }
        else
        {
            traj_xv_goal = damper(traj_xv_goal, x_target_v, 0.1f);
            traj_yv_goal = damper(traj_yv_goal, y_target_v, 0.1f);
        }

        spring_character_update(trajx, trajxv, trajxa, traj_xv_goal, halflife, dt);
        spring_character_update(trajy, trajyv, trajya, traj_yv_goal, halflife, dt);

        spring_character_predict(predx, predxv, predxa, PRED_MAX, trajx, trajxv, trajxa, traj_xv_goal, halflife, dt * PRED_SUB);
        spring_character_predict(predy, predyv, predya, PRED_MAX, trajy, trajyv, trajya, traj_yv_goal, halflife, dt * PRED_SUB);

        trajx_prev[0] = trajx;
        trajy_prev[0] = trajy;

        std::vector<dfm2::CVec2d> vec_pos2;
        vec_pos2.clear();
        std::vector<double> future_traj;
        future_traj.clear();

        for (int i = 0; i < TRAJ_MAX - TRAJ_SUB; i += TRAJ_SUB)
        {
            std::vector start = { trajx_prev[i + 0], trajy_prev[i + 0] };
            std::vector stop = { trajx_prev[i + TRAJ_SUB], trajy_prev[i + TRAJ_SUB] };

            if (i == 0) {
                draw_black_sphere(start);
                //std::cout << "root pos : " << start[0] << " - " << start[1] << std::endl;
            }
            else
            {
                dfm2::CVec2d tempt(start[0], start[1]);
                vec_pos2.push_back(tempt);
                draw_blue_sphere(start);
            }
            //DrawLineV(start, stop, BLUE);

            if (i + TRAJ_SUB == TRAJ_MAX - TRAJ_SUB)
            {
                dfm2::CVec2d tempt(stop[0], stop[1]);
                vec_pos2.push_back(tempt);
                draw_blue_sphere(stop);
            }
        }
        std::reverse(vec_pos2.begin(), vec_pos2.end());
    
        for (int i = 1; i < PRED_MAX; i += 1)
        {
            std::vector start = { predx[i + 0], predy[i + 0] };
            std::vector stop = { predx[i - 1], predy[i - 1] };

            //DrawLineV(start, stop, MAROON);
            dfm2::CVec2d tempt(start[0], start[1]);
            vec_pos2.push_back(tempt);
            future_traj.push_back(start[0] - trajx_prev[0]);
            future_traj.push_back(start[1] - trajy_prev[0]);
            draw_red_sphere(start);
        }


        speed_x = (predx[1] - trajx_prev[0]) / (dt * 20);
        speed_y = (predy[1] - trajy_prev[0]) / (dt * 20);
        if (facing_control_mode == false)
            viewer_source.view_rotation->Rot_Camera(-static_cast<float>(gamepad2_x), -static_cast<float>(gamepad2_y));


        //NN input are here  *** (ßÍß) ***
        float goal_x = 0.0f;
        float goal_y = 0.0f;
        if (facing_control_mode == false) {
            dfm2::CVec2d goal_dirZ(predx[1] - trajx_prev[0], predy[1] - trajy_prev[0]);
            goal_dirZ.normalize();
            face_dirZ.normalize();
            goal_x = damper(face_dirZ.x, goal_dirZ.x, 0.1);
            goal_y = damper(face_dirZ.y, goal_dirZ.y, 0.1);
            if ((goal_x >= -1) && (goal_x <= 1))
            {
                face_dirZ.x = goal_x; // get rid of initial broken data, fix it later
            }
            if ((goal_y >= -1) && (goal_y <= 1))
            {
                face_dirZ.y = goal_y;
            }
        }
        else {
            dfm2::CVec2d goal_dirZ(gamepad2_x, gamepad2_y);
            goal_dirZ.normalize();
            face_dirZ.normalize();
            goal_x = damper(face_dirZ.x, goal_dirZ.x, 0.05);
            goal_y = damper(face_dirZ.y, goal_dirZ.y, 0.05);
            if ((goal_x >= -1) && (goal_x <= 1))
            {
                face_dirZ.x = goal_x; // get rid of initial broken data, fix it later
            }
            if ((goal_y >= -1) && (goal_y <= 1))
            {
                face_dirZ.y = goal_y;
            }
        }


        //face_dirZ.x = goal_x;
        //face_dirZ.y = goal_y;
        face_dirZ.normalize();
        //std::cout << "face_dir" << face_dirZ[0] << "-" << face_dirZ[1] << std::endl;
        //

        ::glLineWidth(1);
        ::glColor3d(0, 0, 0);
        ::glBegin(GL_LINES);
        ::glVertex3d(trajx_prev[0], 0.1f, trajy_prev[1]);
        ::glVertex3d(trajx_prev[0] + face_dirZ[0] * 5, 0.1f, trajy_prev[1] + face_dirZ[1] * 5);
        ::glEnd();

        dfm2::CVec2d root_pos2(trajx_prev[0], trajy_prev[0]);
        // std::cout <<   "traj size : " << vec_pos2.size() << std::endl;
                //  BVH test

        int jump_frame = Pose_Match_Best(posematch_date, future_traj, posematch_date.size(), 10);
        SetPose_BioVisionHierarchy_Rotate(aBone, aChannelRotTransBone, vec_bvh_time_series_data.data() + jump_frame * nch, root_pos2);
        //SetPose_BioVisionHierarchy_Rotate(aBone, aChannelRotTransBone, vec_bvh_time_series_data.data() + iframe * nch, root_pos2);
        dfm2::opengl::DrawBone_Octahedron(
            aBone,
            5, -1,
            0.1, 1.0);
        iframe = (iframe + 1) % nframe;
        // 
        DrawFloorShadow(aBone, floor, +0.1);

        floor.draw_checkerboard();




        // GUI *******
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 1. Show the big demo window (Most of the sample code is in ImGui::ShowDemoWindow()! You can browse its code to learn more about Dear ImGui!).
        {
            //static float f = 0.0f;
            ImGui::Begin("Guide");
            ImGui::Text("Controller User");
            ImGui::BulletText("Leftstick       - Control Moving Direction");
            ImGui::BulletText("Rightstick      - Control Camera Direction");
            ImGui::BulletText("LT + Rightstick - Control Facing Direction");
            ImGui::BulletText("RT (HOLD)       - Switch to Run");

            ImGui::Separator();
            ImGui::Text("Keyboard User");
            ImGui::BulletText(" W S A D (HOLD)                 - Control Moving Direction");
            ImGui::BulletText(" ALT (HOLD) + Left Click (HOLD) - Control Camera Direction");
            ImGui::BulletText(" F                              - Switch to Run/Walk");
            ImGui::BulletText(" X                              - Stop Character");
            ImGui::End();

            ImGui::Begin("Control Panel");                       
            ImGui::Checkbox("XBOX Gamepad", &check_controller);
            ImGui::Text("Buttons"); ImGui::SameLine();
            ImGui::Checkbox("X", &buttonX); ImGui::SameLine();
            ImGui::Checkbox("Y", &buttonY); ImGui::SameLine();
            ImGui::Checkbox("A", &buttonA); ImGui::SameLine();
            ImGui::Checkbox("B", &buttonB); ImGui::SameLine();
            ImGui::Separator();
            ImGui::Text("Trajectory smoothness");
            ImGui::SliderFloat("Traj Half-life", &halflife, 0.0f, 0.9f);
            ImGui::Separator();
            ImGui::Text("Current Direction");
            ImGui::Text("dir x: %f ; dir y: %f", goal_x, goal_y);
            ImGui::Text("Current Speed");               // Display some text (you can use a format strings too)
            ImGui::Text("dir x: %f ; dir y: %f", speed_x, speed_y);
            ImGui::Separator();

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(viewer_source.window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        //glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        //glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        // GUI  *********

        viewer_source.SwapBuffers();
        //glfwSwapBuffers(viewer_source.window);
        glfwPollEvents();
    }


    glfwDestroyWindow(viewer_source.window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}