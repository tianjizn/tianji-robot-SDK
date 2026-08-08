#include "FXToolDynIdent.h"
#include <unistd.h>
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <atomic>
#include <array>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cmath>
#include <mutex>

namespace fs = std::filesystem;

static std::mutex g_task_mutex;
static int g_active_task_count = 0;



struct TaskContext {
    std::atomic<ToolDynTaskStatus> status{TASK_IDLE};
    std::atomic<bool> stop_requested{false};
    std::atomic<int> progress{0};
    std::thread worker;
    LoadDynamicPara result;
    std::atomic<FXFuncReturn> error_code{FUNC_RET_SUCCESS};
    std::mutex mtx;
};



double extractValueFromItem(const std::string& item) {
    size_t pos = item.find(' ');
    if (pos == std::string::npos) return 0.0;
    std::string numStr = item.substr(pos + 1);
    try {
        return std::stod(numStr);
    }
    catch (...) {
        return 0.0;
    }
}


std::vector<std::vector<double>> readCSV1(const std::string& filename) {
    std::vector<std::vector<double>> data; 
    std::ifstream file(fs::u8path(filename));
    if (!file.is_open()) {
        std::cerr << "cannot open file: " << filename << std::endl;
        return data;
    }

    std::string line;
    int lineNum = 0;
    bool firstRow = true;

    
    const char keys[] = { 'X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V', 'W' };
    const int COL_COUNT = 9;

    while (std::getline(file, line)) {
        lineNum++;

      
        if (firstRow) {
            firstRow = false;
            continue;
        }
        if (line.empty()) continue;
        std::vector<double> row(COL_COUNT, 0.0);
        std::stringstream ss(line);
        std::string item;
        while (std::getline(ss, item, '$')) {
            if (item.empty()) continue;
            char key = item[0];  
            double value = extractValueFromItem(item);
            for (int i = 0; i < COL_COUNT; ++i) {
                if (keys[i] == key) {
                    row[i] = value;
                    break;
                }
            }
        }

        data.push_back(row);
    }

    file.close();
    std::cout << "read : " << data.size() << " rows, "
        << (data.empty() ? 0 : data[0].size()) << "cols" << std::endl;
    return data;
}

static fs::path resolve_tool_data_dir(const std::string& traj_file_path) {
    try {
        fs::path p = fs::absolute(fs::u8path(traj_file_path));
        for (fs::path cur = p; !cur.empty() && cur != cur.parent_path(); cur = cur.parent_path()) {
            if (cur.filename() == "ToolDynIdentData") {
                return cur;
            }
        }
    } catch (...) {
    }
    return fs::u8path("./C_SDK/FXConfig/ToolDynIdentData");
}

static int thread0(int loadcode, const fs::path& data_dir,
                   std::atomic<bool>& stop_flag, TaskContext& task, std::atomic<bool>& thread_err)
{
    FX_CHAR buff2[116]={0};
    fs::path finalPath = data_dir / (loadcode == 0 ? "NoLoadData.csv" : "LoadData.csv");
    bool autoCreateDir = true;

    if (autoCreateDir) {
        if (!data_dir.empty() && !fs::exists(data_dir)) {
            if (!fs::create_directories(data_dir)) {
                std::cerr << "err: Can't create directory " << data_dir << std::endl;
                task.error_code.store(FUNC_RET_LOAD_SAMPLE_SAVE_DIR_NOT_EXIST);
                thread_err.store(true);
                return -1;
            }
        }
    }

    std::ofstream file(finalPath);
    if (!file.is_open()) {
        std::cerr << "err: Can't open file " << finalPath << std::endl;
        task.error_code.store(FUNC_RET_LOAD_SAMPLE_SAVE_DIR_NOT_EXIST);
        thread_err.store(true);
        return -1;
    }
    while (!stop_flag.load())
    {

        FX_L1_Fbk_GetUserData((void*)buff2);
        double tag = *((FX_INT32*)&buff2[112]);
        int precision= 6;
        file << std::fixed << std::setprecision(precision);
        for(int i=0; i<14; ++i){
        double num2 = *((double*)&buff2[i*8]);  
        file<< num2; 
        file << ",";               
        }
        file << tag;
        file << "\n";
        usleep(2000);

    }
    file.close();
    usleep(2000); 
    return 0; 
}

int Position_Sampling(int arm, char *file_path, int loadcode, TaskContext& task) {
    // ...
    std::atomic<bool> stop_flag(false); 
    std::atomic<bool> thread_err(false);
    bool firstRun = true;               
    std::vector<std::vector<double>>CSV2array;
    fs::path data_dir = resolve_tool_data_dir(file_path);
    std::thread thread_Fbk([&]() {
        thread0(loadcode, data_dir, stop_flag, task, thread_err);
    });
    // Exception safety: if anything between here and the explicit join throws,
    // join thread_Fbk during stack unwinding so it is never destroyed while
    // still joinable (which would call std::terminate).
    struct FbkJoinGuard {
        std::thread& th;
        std::atomic<bool>& flag;
        ~FbkJoinGuard() { flag.store(true); if (th.joinable()) th.join(); }
    } fbk_guard{thread_Fbk, stop_flag};
    auto cleanup_and_exit = [&](int ret_code) -> int {
    stop_flag = true;
    if (thread_Fbk.joinable()) {
        thread_Fbk.join();
    }
    return ret_code;
    };
    std::string LoadIdenTraj(file_path);
    
    CSV2array= readCSV1(LoadIdenTraj);
    if(CSV2array.empty()){
        task.error_code.store(FUNC_RET_LOAD_TRAJECTORY_FILE_OR_DIR_INVALID);
        printf("LoadIdenTraj is null\n");
    }
    FX_L1_Fbk_ResetUserDataSet();
    usleep(2000);
    if(arm==0){
        if (FX_L1_Fbk_RegisterUserDataSet((char*)"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_Pos", FX_double, 0, 7) != FUNC_RET_SUCCESS
            || FX_L1_Fbk_RegisterUserDataSet((char*)"ROBOT_RT.m_ARMS[0].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor", FX_double, 0, 7) != FUNC_RET_SUCCESS
            || FX_L1_Fbk_RegisterUserDataSet((char*)"ROBOT_RT.m_ARMS[0].m_ARM_IN.m_ARM_CMD_Tag",FX_int,0, 1)!= FUNC_RET_SUCCESS )
        {                                        
            printf("Failed to register user data\n");
            task.error_code.store(FUNC_RET_LOAD_SAMPLE_CONFIG_FAILED);
            return cleanup_and_exit(0);
        }
        printf("Config user sample arm 0 data success\n");         
    }
    else{        
        if (FX_L1_Fbk_RegisterUserDataSet((char*)"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_Pos", FX_double, 0, 7) != FUNC_RET_SUCCESS
            || FX_L1_Fbk_RegisterUserDataSet((char*)"ROBOT_RT.m_ARMS[1].m_ARM_OUT.m_ARM_FBK_Joint_SensorTor", FX_double, 0, 7) != FUNC_RET_SUCCESS
            || FX_L1_Fbk_RegisterUserDataSet((char*)"ROBOT_RT.m_ARMS[1].m_ARM_IN.m_ARM_CMD_Tag",FX_int,0, 1)!= FUNC_RET_SUCCESS)
        {
            printf("Failed to register user data\n");
            task.error_code.store(FUNC_RET_LOAD_SAMPLE_CONFIG_FAILED);
            return cleanup_and_exit(0);
        }
        printf("Config user sample arm1 data success\n"); 
    }
    double joints[7] = { 0,0,0,0,0,0,0 };
    int tag= 0;
    int rows= CSV2array.size();
    printf("rang=%d",rows);
    unsigned int system_errorcode = 0;    ///< Last system error code
    FXStateType obj_state = FX_STATE_UNKNOWN;

    double vel_ratio = 50.0;              ///< Velocity ratio for position mode
    double acc_ratio = 50.0;              ///< Acceleration ratio for position mode
    FXObjType ARM= FX_OBJ_ARM0;
    if(arm==0){
      ARM= FX_OBJ_ARM0;
      printf("arm is %d",arm);  
    } 
    else if (arm==1){
      ARM= FX_OBJ_ARM1;  
      printf("arm is %d",arm); 
    } 
    else{
        printf("robot arm set err");
    }
    obj_state = FX_L1_Fbk_CurrentState(ARM);
    if (obj_state == FX_STATE_ERROR)
    {
        if (FX_L1_State_ResetError(ARM, 1000, &system_errorcode) == FUNC_RET_SUCCESS)
        {
            printf("Reset arm0 error success, arm1 is now in STATE_IDLE state\n");
        }
        else
        {
            printf("Failed to reset arm1 error, errorcode = 0x%08x\n", system_errorcode);
        }
    }
    usleep(2000);   
    if (FX_L1_State_SwitchToPositionMode(ARM, 2000, vel_ratio, acc_ratio) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm1 to STATE_POSITION state\n");
        task.error_code.store(FUNC_RET_LOAD_TRAJECTORY_OPERATION_FAILED);
        return cleanup_and_exit(0);
    }
    
    printf("Arm1 is in STATE_POSITION state now\n");
    int cols;
    for(int i=1; i<rows-1 && !task.stop_requested.load(); ++i){
        if(thread_err.load()){
            printf("Sampling thread error, task.error_code=(%d),aborting trajectory\n", task.error_code.load() );
            break;
        }
        cols= CSV2array[i].size();
        if(cols<9){
            printf("%d rows has only %d colus ",i,cols);
            usleep(2000);
            continue;
        }
        joints[0] = CSV2array[i][0];
        joints[1] = CSV2array[i][1];
        joints[2] = CSV2array[i][2];
        joints[3] = CSV2array[i][3];
        joints[4] = CSV2array[i][4];
        joints[5] = CSV2array[i][5];
        joints[6] = CSV2array[i][6];
        tag= CSV2array[i][8];
        
        int num_integer = static_cast<int>((static_cast<double>(i) / rows) * 100);
        double num_decimal= (static_cast<double>(i) / rows) * 100- num_integer;
        int num= 0;
        if(num_decimal>0.5){
            num= num_integer+ 1;
        }
        else{
            num= num_integer;
        }
        task.progress.store(num);
        if (FX_L1_Runtime_SetJointPosCmd(1, ARM, joints) != FUNC_RET_SUCCESS 
        || FX_L1_Runtime_SetTag(2, ARM, tag) != FUNC_RET_SUCCESS  )
        {
            printf("Failed to set arm1's target position\n");
            task.error_code.store(FUNC_RET_LOAD_TRAJECTORY_OPERATION_FAILED);
            return cleanup_and_exit(0);
        }
        if(firstRun){
            sleep(2);
            firstRun= false;
        }
        usleep(2000);
    }
    stop_flag.store(true);
    if (thread_Fbk.joinable()) {
        thread_Fbk.join();
    }
    if (FX_L1_State_SwitchToIdle(ARM, 1000) != FUNC_RET_SUCCESS)
    {
        printf("Failed to transfer arm0 to STATE_IDLE state\n");
        // return 0;
    }
    FX_L1_Fbk_ResetUserDataSet();
    return 1;
}
/**
 * @brief Execute the load identification main workflow
 * @param arm           Arm number: 0=left arm, 1=right arm
 * @param traj_file_path Trajectory file path (.fmv format)
 * @param loadcode      Load status: 0=no load, 1=with load
 * @return 0 on success, non-zero on failure
 */
int run_load_sampling(int arm, const char* traj_file_path, 
                            int loadcode, TaskContext& task) {                           
    task.status.store(TASK_SAMPLING);
    int ret = Position_Sampling(arm, (char*)traj_file_path, loadcode, task);
    // task.status.store(TASK_SAMPLING);                          
    if (ret == 0 && !task.stop_requested.load()){
        fprintf(stderr, "Position_Sampling failed\n");
        task.status.store(TASK_ERROR);
        return -1;
    }
    if(task.stop_requested.load()){
        printf("stop is requested");
        return -1;
    }
    task.status.store(TASK_DONE);
    return 0;
}
// ///////////////////////////////////////explosure api/////////////////////////////////////////////////////////////////////////

TaskHandle FX_ToolDyn_Start_Sampling(int arm, int loadcode,
                         const char* traj_file_path) {
    {
        std::lock_guard<std::mutex> lock(g_task_mutex);
        if (g_active_task_count > 0) {
            fprintf(stderr, "Error: Another identification task is already running.\n");
            return nullptr;
        }
        ++g_active_task_count;
    }  
    printf("read file:%s\n", traj_file_path ? traj_file_path : "(null)");
    auto* task = new TaskContext();
    std::string path_copy = traj_file_path ? traj_file_path : "";
    task->worker = std::thread([task, arm, path_copy, loadcode]() {
        struct CounterGuard {
                    ~CounterGuard() {
                    std::lock_guard<std::mutex> lock(g_task_mutex);
                    --g_active_task_count;
                }
            } guard;
        try {
            run_load_sampling(arm, path_copy.c_str(), loadcode, *task);
        } catch (...) {
            task->status.store(TASK_ERROR);
        }
    });

    if(!task->worker.joinable()){
        printf(" fail to start identification");
        return nullptr;
    }
    else{
        // task->status.store(TASK_SAMPLING);
        return static_cast<TaskHandle>(task);
    }
                              
}

int FX_ToolDyn_Get_Sampling_Status(TaskHandle handle, ToolDynTaskStatus* status, int* progress, FXFuncReturn* error_code){
    auto* ctx = static_cast<TaskContext*>(handle);
    if(!ctx){
        printf("fail to get identification status");
     return -1;   
    } 
    *status= ctx->status;
    *progress= ctx->progress;
    *error_code= ctx-> error_code;
    return 0;

}


int FX_ToolDyn_Stop_Sampling(TaskHandle handle) {
    if (!handle) return -1;
    auto* ctx = static_cast<TaskContext*>(handle);
    ctx->stop_requested.store(true);
    if (ctx->worker.joinable()) {
        ctx->worker.join();
    }
    if (ctx->status.load() != TASK_ERROR) ctx->status.store(TASK_STOPPED);
    return 0;
}

int FX_ToolDyn_Destroy_Sampling(TaskHandle handle){
    if (!handle) return -1;
    auto* ctx = static_cast<TaskContext*>(handle);
    // Ensure the worker is not joinable before deleting: destroying a joinable
    // std::thread calls std::terminate. If stop_identification was not called,
    // join here (the worker exits promptly once stop_requested is observed).
    if (ctx->worker.joinable()) {
        ctx->stop_requested.store(true);
        ctx->worker.join();
    }
    delete ctx;
    return 0;
}

int FX_ToolDyn_Run_Load_Identification(FXFuncReturn* error_code,
                            LoadDynamicPara* out_dynPara,
                            const char* traj_file_path 
                        )
{
    if (!out_dynPara){
        printf("out_dynPara is null\n");
        return -1;   
    }
    // Anchor the identification input dir to the same absolute ToolDynIdentData
    // folder the sampled data was written to, so OnCalLoadDyn reads the right
    // files regardless of the working directory.
    fs::path data_dir = resolve_tool_data_dir(traj_file_path);
    char UserPath[512];
    snprintf(UserPath, sizeof(UserPath), "%s", data_dir.string().c_str());
    int robot_type= 1;
    int ret = 0;
    ret = OnCalLoadDyn(out_dynPara, robot_type, UserPath);
    if (ret != 0) {
        printf("OnCalLoadDyn failed with code %d\n", ret);
        *error_code = static_cast<FXFuncReturn>(-1030-ret);
        return -1;
    }
    else{
        printf("OnCalLoadDyn success\n");
    }
    return 0;
}
