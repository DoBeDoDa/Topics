#include <cstdint>  // std::uint8_t
#include <vector>

#ifndef HRSDK_HRSDK_H_
#define HRSDK_HRSDK_H_

#ifdef _WIN32
#ifdef HRSDK_HRSDK_H_
#define HRSDK_API __declspec(dllexport)
#else
#define HRSDK_API __declspec(dllimport)
#endif
#else
#define HRSDK_API
#define __stdcall
#endif

typedef int HROBOT;
#ifdef __cplusplus
extern "C" {
#endif
enum CommandType {
	kGet = 0,
	kSet,
	kMonitorSet,
};
enum ConnectionLevels {
	kVerMismatch = -2,
	kDisconnection = -1,
	kMonitor = 0,
	kController
};

enum OperationModes {
	kManual = 0,
	kAuto
};

enum LogLevels {
	kNone = 0,
	kInfo,
	kSetCommand,
	kConsole,
	kSave,
};

enum LogMessage {
	kNormal = 0,
	kWrite,
};

enum Connect {
	INVALID_CALLBACK = -2,
	CONNECT_SERVER_FAILED = -3,
	VERSION_MISMATCH = -4
};

enum SpaceOperationTypes {
	kCartesian = 0,
	kJoint,
	kTool,
	kExt
};

enum SpaceOperationDirection {
	kPositive = 1,
	kNegative = -1,
};

enum JointCoordinates {
	kJoint1 = 0,
	kJoint2,
	kJoint3,
	kJoint4,
	kJoint5,
	kJoint6
};

enum CartesianCoordinates {
	kCartesianX = 0,
	kCartesianY,
	kCartesianZ,
	kCartesianA,
	kCartesianB,
	kCartesianC
};

enum ToolCoordinates {
	kTx = 0,
	kTy,
	kTz,
	kRTx,
	kRTy,
	kRTz
};

enum RobotMotionStatus {
	kServoOff = 0,
	kIdle = 1,
	kRunning,
	kHold,
	kDelay,
	kWait
};

enum Welder {
	Binzel_MIG_Arc_350_RS,
	Hero_TIG_RA300,
	General_MIG_Volts_WFS,
	General_MIG_Volts_Amps,
	General_TIG_Amps,
	General_TIG_Amps_WFS,
};

struct WeldSystemParameter {
	Welder welder;
	int weld_enable;
	int gas_purge_input_index;
	int gas_purge_input_type;
	double gas_purge_time;
	int torch_collision_input_index;
	int torch_collision_input_type;
	int torch_collision_input_switch_type;
	int wirestick_detection;
	double arc_start_error_time;
	double arc_loss_error_time;
	double arc_detect_time;
	int arc_retry_count;
	double arc_retry_wire_retract_time;
	int ALC_enable;
};
enum WeldModeHeroTIG {
	Hero_TIG_DC,
	Hero_TIG_AC_Hard,
	Hero_TIG_AC_Standard,
	Hero_TIG_AC_Soft,
	Hero_TIG_AC_Mix,
	Hero_TIG_JobMode
};
enum MechanicType {
	kMechanicNone = -1,
	kMechanicSlider,
	kMechanicOnePositioner,
	kMechanicTwoPositioner,
	KMechanicTypeCount,
};
enum WeldMode {
	Standard,
	PulsedArc,
	JobMode,
	LowSpatterMode
};
struct WeldProcedureParameter {
	int schedule_num;
	WeldMode weld_mode;
	WeldModeHeroTIG tig_weld_mode;
	int weld_program;
	int synergy_curve;
	double runin_time;
	double runin_voltage;
	double runin_current;
	double runin_wire_speed;
	double burnback_time;
	double burnback_voltage;
	double burnback_current;
	double burnback_wire_speed;
	double crater_time;
	double crater_voltage;
	double crater_current;
	double crater_wire_speed;
	double preflow_time;
	double postflow_time;
	double tig_rise_time;
	double tig_arc_current;
	double tig_arc_time;
	double tig_forward_blowing_time;
	double tig_start_current;
	double tig_ac_dc_mix_freq;
	double tig_ac_frequence;
	double tig_negative_current_duty_cycle;
	double tig_negative_current_percentage;
	double tig_impulse_on;
	double tig_backward_blowing_time;
	double tig_final_current;
	double tig_drop_time;
	int torch_mode;
	int job_number;
};
struct WeldScheduleParameter {
	double voltage;
	double current;
	double wire_speed;
	double time;
	double tig_back_ground_main_current;
	double tig_impulse_freq;
	double tig_impulse_work_cycle;
	double tig_main_arc_current;
	double speed;
};
#define DWELL_MAX_TYPE_SIZE	2
typedef struct {
	short weaving_mode;
	short dwell_mode;
	double frequency;		// unit:Hz
	double amplitude;		// unit:mm
	double dwell_time[DWELL_MAX_TYPE_SIZE];	// unit:sec
	double elevation_angle;	// unit:deg
	double azimuth_angle;	// unit:deg
	double center_rise;		// unit:mm
	double radius;			// unit:mm
	double L_angle;			// unit:deg
	unsigned char weaving_blend;
	short  peak_digital_output_number;
	double peak_output_pulse;	// unit:sec
	double peak_output_shift;	// unit:sec
} ROBOT_WEAVING_CONDITION;
struct RefreshUIParameter {
	unsigned char  base_;
	unsigned char  counter;
	unsigned char  timer;
	unsigned char  user_alarm;
	unsigned char  pns;
	unsigned char  dio_setting;
	unsigned char  external_axis;
	unsigned char  fieldbus;
	unsigned char  moudle_io;
	unsigned char  pr;
	unsigned char  payload;
	unsigned char  ref_position;
	unsigned char  socket_info;
	unsigned char  tool;
	unsigned char  weave;
	unsigned char  dio_comment;
};


/* Connection Command */
typedef void(__stdcall *callback_function)(uint16_t, uint16_t, uint16_t *, int);
HRSDK_API HROBOT __stdcall open_connection(const char *address, int level, callback_function f);
HRSDK_API HROBOT __stdcall open_connection_no_callback(const char *address, int level);
HRSDK_API HROBOT __stdcall open_multi_connection(const char *address, int Mode, callback_function callback_func, int hrs_cnt);
HRSDK_API int __stdcall disconnect(HROBOT s);
HRSDK_API void __stdcall close_connection(HROBOT s);
HRSDK_API int __stdcall set_connection_level(HROBOT s, int Mode);
HRSDK_API int __stdcall get_connection_level(HROBOT s);
HRSDK_API int __stdcall get_hrsdk_version(char *version);
HRSDK_API int __stdcall set_log_level(HROBOT robot_handle, LogLevels log_level);
HRSDK_API LogLevels __stdcall get_log_level(HROBOT robot_handle);

/* Register Command */
HRSDK_API int __stdcall set_timer(HROBOT s, int index, int time);
HRSDK_API int __stdcall get_timer(HROBOT s, int index);
HRSDK_API int __stdcall set_timer_start(HROBOT s, int index);
HRSDK_API int __stdcall set_timer_stop(HROBOT s, int index);
HRSDK_API int __stdcall set_timer_name(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall get_timer_name(HROBOT s, int index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall get_timer_status(HROBOT s, int index);
HRSDK_API int __stdcall set_counter(HROBOT s, int index, int co);
HRSDK_API int __stdcall get_counter(HROBOT s, int index);
HRSDK_API int __stdcall set_pr_type(HROBOT s, int prNum, int coorType);
HRSDK_API int __stdcall get_pr_type(HROBOT s, int prNum);
HRSDK_API int __stdcall set_pr_coordinate(HROBOT s, int prNum, double *coor);
HRSDK_API int __stdcall get_pr_coordinate(HROBOT s, int pr, double *coor);
HRSDK_API int __stdcall set_pr_tool_base(HROBOT s, int prNum, int toolNum, int baseNum);
HRSDK_API int __stdcall get_pr_tool_base(HROBOT s, int pr, int *tool_base);
HRSDK_API int __stdcall set_pr(HROBOT s, int prNum, int coorType, double *coor, double *ext_pos, int tool, int base);
HRSDK_API int __stdcall get_pr(HROBOT s, int pr_num, int *coor_type, double *coor, double *ext_pos, int *tool, int *base);
HRSDK_API int __stdcall remove_pr(HROBOT s, int pr_num);
HRSDK_API int __stdcall set_pr_comment(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall get_pr_comment(HROBOT s, int index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall get_counter_name(HROBOT s, int index, wchar_t *name, int arr_size);
HRSDK_API int __stdcall set_counter_name(HROBOT s, int index, wchar_t *name);
HRSDK_API int __stdcall set_real(HROBOT s, int index, double co);
HRSDK_API double __stdcall get_real(HROBOT s, int index);
HRSDK_API int __stdcall get_real_name(HROBOT s, int index, wchar_t *name, int arr_size);
HRSDK_API int __stdcall set_real_name(HROBOT s, int index, wchar_t *name);
HRSDK_API int __stdcall set_string(HROBOT s, int index, wchar_t *value);
HRSDK_API int __stdcall get_string(HROBOT s, int index, wchar_t *value, int arr_size);
HRSDK_API int __stdcall get_string_name(HROBOT s, int index, wchar_t *name, int arr_size);
HRSDK_API int __stdcall set_string_name(HROBOT s, int index, wchar_t *name);

/* System Variable Command */
HRSDK_API int __stdcall set_acc_dec_ratio(HROBOT s, int acc);
HRSDK_API int __stdcall get_acc_dec_ratio(HROBOT s);
HRSDK_API int __stdcall set_acc_time(HROBOT s, double value);
HRSDK_API double __stdcall get_acc_time(HROBOT s);
HRSDK_API int __stdcall set_ptp_speed(HROBOT s, int vel);
HRSDK_API int __stdcall get_ptp_speed(HROBOT s);
HRSDK_API int __stdcall set_lin_speed(HROBOT s, double vel);
HRSDK_API double __stdcall get_lin_speed(HROBOT s);
HRSDK_API int __stdcall set_override_ratio(HROBOT s, int vel);
HRSDK_API int __stdcall get_override_ratio(HROBOT s);
HRSDK_API int __stdcall get_robot_id(HROBOT s, char *robot_id);
HRSDK_API int __stdcall set_robot_id(HROBOT s, char *robot_id);
HRSDK_API int __stdcall set_smooth_length(HROBOT s, double r);
HRSDK_API int __stdcall get_alarm_code(HROBOT s, int &count, uint64_t *alarm_code);
HRSDK_API int __stdcall get_alarm_log_count(HROBOT s, int &size);
HRSDK_API int __stdcall get_alarm_log_msg(HROBOT s, int idx, wchar_t *msg);

/* Input and Output Command */
HRSDK_API int __stdcall get_digital_input(HROBOT s, int index);
HRSDK_API int __stdcall get_digital_output(HROBOT s, int index);
HRSDK_API int __stdcall set_digital_output(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_robot_input(HROBOT s, int index);
HRSDK_API int __stdcall get_robot_output(HROBOT s, int index);
HRSDK_API int __stdcall set_robot_output(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_valve_output(HROBOT s, int index);
HRSDK_API int __stdcall set_valve_output(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_function_input(HROBOT s, int index);
HRSDK_API int __stdcall get_function_output(HROBOT s, int index);
HRSDK_API int __stdcall set_DI_simulation_Enable(HROBOT s, int index, bool v);
HRSDK_API int __stdcall set_DI_simulation(HROBOT s, int index, bool v);
HRSDK_API int __stdcall get_DI_simulation_Enable(HROBOT s, int index);
HRSDK_API int __stdcall set_digital_input_comment(HROBOT s, int di_index, wchar_t *comment);
HRSDK_API int __stdcall get_digital_input_comment(HROBOT s, int di_index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall set_digital_output_comment(HROBOT s, int do_index, wchar_t *comment);
HRSDK_API int __stdcall get_digital_output_comment(HROBOT s, int do_index, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall SyncOutput(HROBOT s, int O_type, int O_id, int on_off, int synMode, int delay, int distance);

/*Module I/O 10/16*/
HRSDK_API int __stdcall get_module_input_config(HROBOT s, int index, bool &sim, bool &value, int &type, int &start, int &end, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall get_module_output_config(HROBOT s, int index, bool &value, int &type, int &start, int &end, wchar_t *comment, int arr_size);
HRSDK_API int __stdcall set_module_input_simulation(HROBOT s, int index, bool enable);
HRSDK_API int __stdcall set_module_input_value(HROBOT s, int index, bool enable);
HRSDK_API int __stdcall set_module_input_start(HROBOT s, int index, int start_number);
HRSDK_API int __stdcall set_module_input_end(HROBOT s, int index, int end_number);
HRSDK_API int __stdcall set_module_input_comment(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall set_module_output_value(HROBOT s, int index, bool enable);
HRSDK_API int __stdcall set_module_output_start(HROBOT s, int index, int start_number);
HRSDK_API int __stdcall set_module_output_end(HROBOT s, int index, int end_number);
HRSDK_API int __stdcall set_module_output_comment(HROBOT s, int index, wchar_t *comment);
HRSDK_API int __stdcall set_module_input_type(HROBOT s, int index, int type);
HRSDK_API int __stdcall set_module_output_type(HROBOT s, int index, int type);
HRSDK_API int __stdcall save_module_io_setting(HROBOT s);

/* Coordinate System Command */
HRSDK_API int __stdcall set_base_number(HROBOT s, int state);
HRSDK_API int __stdcall get_base_number(HROBOT s);
HRSDK_API int __stdcall define_base(HROBOT s, int baseNum, double *coor);
HRSDK_API int __stdcall get_base_data(HROBOT s, int num, double *coor);
HRSDK_API int __stdcall get_moving_base_data(HROBOT s, double *coor);
HRSDK_API int __stdcall base_calibration(HROBOT s, int calibrate_type, double *p0coor, double *p1coor, double *p2coor, double *result_coor);
HRSDK_API int __stdcall set_tool_number(HROBOT s, int num);
HRSDK_API int __stdcall get_tool_number(HROBOT s);
HRSDK_API int __stdcall define_tool(HROBOT s, int toolNum, double *coor);
HRSDK_API int __stdcall get_tool_data(HROBOT s, int num, double *coor);
HRSDK_API int __stdcall tool_calibration(HROBOT s, int calibrate_type, double *p0coor, double *p1coor, double *p2coor, double *p3coor, double *result_coor);
HRSDK_API int __stdcall set_home_point(HROBOT s, double *joint);
HRSDK_API int __stdcall get_home_point(HROBOT s, double *joint);
HRSDK_API int __stdcall set_ext_home_point(HROBOT s, double *ext_pos);
HRSDK_API int __stdcall get_ext_home_point(HROBOT s, double *ext_pos);
HRSDK_API int __stdcall get_previous_pos(HROBOT s, double *joint);
HRSDK_API int __stdcall get_previous_extpos(HROBOT s, double *ext_pos);
HRSDK_API int __stdcall enable_joint_soft_limit(HROBOT v, bool enable);
HRSDK_API int __stdcall enable_cart_soft_limit(HROBOT s, bool enable);
HRSDK_API int __stdcall set_joint_soft_limit(HROBOT s, double *low_limit, double *high_limit);
HRSDK_API int __stdcall set_cart_soft_limit(HROBOT s, double *low_limit, double *high_limit);
HRSDK_API int __stdcall get_joint_soft_limit_config(HROBOT s, bool &enable, double *low_limit, double *high_limit);
HRSDK_API int __stdcall get_cart_soft_limit_config(HROBOT s, bool &enable, double *low_limit, double *high_limit);
HRSDK_API int __stdcall confirm_home_point(HROBOT s);

/* Task Command */
HRSDK_API int __stdcall set_rsr(HROBOT s, char *filename, int index);
HRSDK_API int __stdcall get_rsr_prog_name(HROBOT s, int rsr_index, char *file_name);
HRSDK_API int __stdcall remove_rsr(HROBOT s, int index);
HRSDK_API int __stdcall ext_task_start(HROBOT s, int mode, int select);
HRSDK_API int __stdcall task_start(HROBOT s, char *task_name);
HRSDK_API int __stdcall task_hold(HROBOT s);
HRSDK_API int __stdcall task_continue(HROBOT s);
HRSDK_API int __stdcall task_abort(HROBOT s);
HRSDK_API int __stdcall task_abort_wait_finish(HROBOT s);
HRSDK_API int __stdcall get_execute_file_name(HROBOT s, char *file_name);
HRSDK_API int __stdcall get_prog_number(HROBOT s);
HRSDK_API int __stdcall get_prog_name(HROBOT s, int file_index, char *file_name);
HRSDK_API int __stdcall task_skip_line(HROBOT s, char *task_name);
HRSDK_API int __stdcall set_skip_line(HROBOT s, int line, char *filename);

/* File Command */
HRSDK_API int __stdcall send_file(HROBOT sock, char *from_file_path, char *to_file_path);
HRSDK_API int __stdcall download_file(HROBOT s, char *from_file_path, char *to_file_path);
HRSDK_API int __stdcall delete_file(HROBOT s, char *FilePath);
HRSDK_API int __stdcall delete_folder(HROBOT s, char *FilePath);
HRSDK_API int __stdcall new_folder(HROBOT s, char *FilePath);
HRSDK_API int __stdcall file_rename(HROBOT s, char *oldFilePath, char *newFilePath);
HRSDK_API int __stdcall file_drag(HROBOT s, char *fromFilePath, char *toFilePath);

/* Controller Setting Command */
HRSDK_API int __stdcall get_hrss_mode(HROBOT s);
HRSDK_API int __stdcall set_motor_state(HROBOT s, int state);
HRSDK_API int __stdcall get_motor_state(HROBOT s);
HRSDK_API int __stdcall set_operation_mode(HROBOT s, int mode);
HRSDK_API int __stdcall get_operation_mode(HROBOT s);
HRSDK_API int __stdcall clear_alarm(HROBOT s);
HRSDK_API int __stdcall update_hrss(HROBOT s, char *path);
HRSDK_API int __stdcall set_speed_limit(HROBOT s, bool mode);
HRSDK_API int __stdcall set_language(HROBOT s, int language_number);
HRSDK_API int __stdcall get_controller_time(HROBOT s, int &year, int &month, int &day, int &hour, int &minute, int &second);
HRSDK_API int __stdcall get_robot_info(HROBOT s, int page_sel, int tool_num, int base_num, char *info, bool is_ready);
HRSDK_API int __stdcall mastering(HROBOT s, char *joint, char *type);
HRSDK_API int __stdcall calibration(HROBOT s, char *joint, char *type);
HRSDK_API int __stdcall switch_btn_stop(HROBOT s);
HRSDK_API int __stdcall write_operationhistory(HROBOT s, char *log_message);

/* Jog */
HRSDK_API int __stdcall jog(HROBOT s, int space_type, int index, int dir);
HRSDK_API int __stdcall jog_home(HROBOT s);
HRSDK_API int __stdcall jog_stop(HROBOT s);
HRSDK_API int __stdcall jog_rotation(HROBOT s, int axis, double offset);
HRSDK_API int __stdcall jog_rotation_increment(HROBOT s, int mode, double degree);
HRSDK_API int __stdcall jog_tool_motion(HROBOT s, double *tool_offset, double *tool_offset_target);

/* Motion Command */
HRSDK_API int __stdcall ptp_pos(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ptp_axis(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ptp_rel_pos(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ptp_rel_axis(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ptp_pr(HROBOT s, int mode, int p);
HRSDK_API int __stdcall ptp_point(HROBOT s, int mode, double *axis, double *pos, double *ext);
HRSDK_API int __stdcall ptp_point2(HROBOT s, int mode, double *axis, double *pos, double *ext);
HRSDK_API int __stdcall lin_pos(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall lin_axis(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall lin_rel_pos(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall lin_rel_axis(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall lin_pr(HROBOT s, int mode, double smooth_value, int p);
HRSDK_API int __stdcall spline(HROBOT s, int mode, double smooth_value);
HRSDK_API int __stdcall set_spl(HROBOT s, int coor_type, double *p);
HRSDK_API int __stdcall clear_spl(HROBOT s);
HRSDK_API int __stdcall circ_pos(HROBOT s, int mode, double *p_aux, double *p_end);
HRSDK_API int __stdcall circ_axis(HROBOT s, int mode, double *p_aux, double *p_end);
HRSDK_API int __stdcall circ_pr(HROBOT s, int mode, int p1, int p2);
HRSDK_API int __stdcall motion_hold(HROBOT s);
HRSDK_API int __stdcall motion_continue(HROBOT s);
HRSDK_API int __stdcall motion_abort(HROBOT s);
HRSDK_API int __stdcall motion_delay(HROBOT s, int delay);
HRSDK_API int __stdcall set_command_id(HROBOT s, int id);
HRSDK_API int __stdcall get_command_id(HROBOT s);
HRSDK_API int __stdcall get_command_count(HROBOT s);
HRSDK_API int __stdcall get_motion_state(HROBOT s);
HRSDK_API int __stdcall remove_command(HROBOT s, int num);
HRSDK_API int __stdcall remove_command_tail(HROBOT s, int num);
HRSDK_API int __stdcall motion_reachable(HROBOT s, double *dest_coord, bool &is_reach);
HRSDK_API int __stdcall motion_check_lin(HROBOT s, double *coord1, double *coord2, bool &is_reach);

/* Manipulator Information Command */
HRSDK_API int __stdcall get_encoder_count(HROBOT s, int32_t *EncCount);
HRSDK_API int __stdcall get_current_joint(HROBOT s, double *joint);
HRSDK_API int __stdcall get_current_position(HROBOT s, double *cart);
HRSDK_API int __stdcall get_position_b0t0(HROBOT s, double *pos);
HRSDK_API int __stdcall get_position_info(HROBOT s, double *cart, double *joints, int *encoder, double *ext_pos, int *ext_encoder);
HRSDK_API int __stdcall get_current_rpm(HROBOT s, double *rpm);
HRSDK_API int __stdcall get_current_ext_rpm(HROBOT s, double *rpm);
HRSDK_API int __stdcall get_current_tcp_speed(HROBOT s, double *speed);
HRSDK_API int __stdcall get_device_born_date(HROBOT s, int *YMD);
HRSDK_API int __stdcall get_operation_time(HROBOT s, int *YMDHm);
HRSDK_API int __stdcall get_mileage(HROBOT s, double *mil);
HRSDK_API int __stdcall get_total_mileage(HROBOT s, double *tomil);
HRSDK_API int __stdcall get_utilization(HROBOT s, int *utl);
HRSDK_API int __stdcall get_utilization_ratio(HROBOT s, double &ratio);
HRSDK_API int __stdcall get_motor_torque(HROBOT s, double *cur);
HRSDK_API int __stdcall get_robot_type(HROBOT s, char *robType);
HRSDK_API int __stdcall get_hrss_version(HROBOT s, char *ver);
HRSDK_API int __stdcall download_history_zip(HROBOT s, char *toFilePath);

/*User Alarm Setting*/
HRSDK_API int __stdcall get_user_alarm_setting_message(HROBOT s, int num, char *message);
HRSDK_API int __stdcall set_user_alarm_setting_message(HROBOT s, int num, char *message);

/*Payload*/
HRSDK_API int __stdcall get_payload_config(HROBOT s, int index, double *value, char *comment);
HRSDK_API int __stdcall set_payload_config(HROBOT s, int index, double *value, char *comment);
HRSDK_API int __stdcall get_payload_active(HROBOT s, int &index);
HRSDK_API int __stdcall set_payload_active(HROBOT s, int index);

/*DIO Setting*/
HRSDK_API int __stdcall get_digital_setting(HROBOT s, int *index, char *text);
HRSDK_API int __stdcall set_digital_setting(HROBOT s, int *index, char *text);

/*Network Config*/
HRSDK_API int __stdcall set_network_show_msg(HROBOT s, int enable);
HRSDK_API int __stdcall get_network_show_msg(HROBOT s, int &flag);
HRSDK_API int __stdcall network_connect(HROBOT s);
HRSDK_API int __stdcall network_disconnect(HROBOT s);
HRSDK_API int __stdcall network_send_msg(HROBOT s, char *msg);
HRSDK_API int __stdcall network_recieve_msg(HROBOT s, char *msg);
HRSDK_API int __stdcall get_network_config(HROBOT s, int &connect_type, char *ip_addr, int &port, int &bracket_type, int &separator_type, bool &is_format);
HRSDK_API int __stdcall set_network_config(HROBOT s, int connect_type, char *ip_addr, int port, int bracket_type, int separator_type, bool is_format);
HRSDK_API int __stdcall network_change_ip(HROBOT s, int lan_index, int ip_type, char *ip_addr);
HRSDK_API int __stdcall network_get_state(HROBOT s);
HRSDK_API int __stdcall get_network_config_mask(HROBOT s, int &connect_type, char *ip_addr, char *subnet_mask, int &port, int &bracket_type, int &separator_type, bool &is_format);
HRSDK_API int __stdcall set_network_config_mask(HROBOT s, int connect_type, char *ip_addr, char *subnet_mask, int port, int bracket_type, int separator_type, bool is_format);
HRSDK_API int __stdcall network_change_ip_mask(HROBOT s, int lan_index, int ip_type, char *ip_addr, char *subnet_mask);

/* External Axis Setting*/
HRSDK_API int __stdcall get_ext_axis_setting(HROBOT s, int index, bool &enable, int &mode, double &high_limit, double &low_limit);
HRSDK_API int __stdcall set_ext_axis_setting(HROBOT s, int index, bool enable, int mode, double high_limit, double low_limit);
HRSDK_API int __stdcall get_ext_axis_setting_advanced(HROBOT s, int index, int &type, bool &math, bool &continuous, int *int_value, double *double_value);
HRSDK_API int __stdcall set_ext_axis_setting_advanced(HROBOT s, int index, int type, bool math, bool continuous, int *int_value, double *double_value);
HRSDK_API int __stdcall ext_ptp(HROBOT s, int index, int direction, double position);
HRSDK_API int __stdcall six_ext_ptp(HROBOT s, int index, int direction, double position);
HRSDK_API int __stdcall ext_mastering(HROBOT s, int index);
HRSDK_API int __stdcall get_current_ext_pos(HROBOT s, double *pos);
HRSDK_API int __stdcall get_current_ext_mode(HROBOT s, char *mode);
HRSDK_API int __stdcall ext_ptp_axis(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ext_ptp_pos(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall ext_lin_axis(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall ext_lin_pos(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall six_ext_lin_pos(HROBOT s, int mode, double smooth_value, double *p);
HRSDK_API int __stdcall ext_asyptp(HROBOT s, int mode, double *p);
HRSDK_API int __stdcall set_ext_driver_limit(HROBOT s, int index, bool enable, bool inverse, int negative_num, int positive_num);
HRSDK_API int __stdcall get_ext_driver_limit(HROBOT s, int index, bool &enable, bool &inverse, int &negative_num, int &positive_num, bool &N_light, bool &P_light);
HRSDK_API int __stdcall get_ext_encoder(HROBOT s, int32_t *EncCount);

/* Home Warning Setting */
HRSDK_API int __stdcall get_home_warning_setting(HROBOT s, double *allow_error_value, double *allow_near_home);
HRSDK_API int __stdcall set_home_warning_setting(HROBOT s, double *allow_error_value, double *allow_near_home);

/* Get All IO*/
HRSDK_API int __stdcall get_DI_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_DI_sim_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_DI_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_DO_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_DO_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_FI_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_FO_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_timer_status_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_timer_value_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_timer_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_counter_value_all(HROBOT s, int *value);
HRSDK_API int __stdcall get_counter_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_fieldbus_rs_srw_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_fieldbus_rs_srr_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_fieldbus_rs_parameter_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_fieldbus_rs_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_system_input_all(HROBOT s, int *value, wchar_t *comment);
HRSDK_API int __stdcall get_system_output_all(HROBOT s, int *value, wchar_t *comment);
HRSDK_API int __stdcall get_MI_config_all(HROBOT s, int *sim, int *value, int *type, int *start, int *end);
HRSDK_API int __stdcall get_MI_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_MO_config_all(HROBOT s, int *value, int *type, int *start, int *end);
HRSDK_API int __stdcall get_MO_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_PR_comment_array(HROBOT s, int *idx, int from_idx, int len, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_SI_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_SI_sim_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_SO_range(HROBOT s, int from_idx, int end_idx, int *value);
HRSDK_API int __stdcall get_SI_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_SO_comment_range(HROBOT s, int from_idx, int end_idx, wchar_t *str, int &next_idx);
HRSDK_API int __stdcall get_PR_array(HROBOT s, int *idx, int from_idx, int len, int *type, double *pos, int *tool, int *base, int &next_idx);
HRSDK_API int __stdcall get_PR_array_v2(HROBOT s, int *idx, int from_idx, int len, int *type, double *pos, int *tool, int *base, int &next_idx);
HRSDK_API int __stdcall get_RI_all(HROBOT s, int *values);
HRSDK_API int __stdcall get_RO_all(HROBOT s, int *values);
HRSDK_API int __stdcall get_VO_all(HROBOT s, int *values);

/*Set All IO*/
HRSDK_API int __stdcall set_DI_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_DI_sim_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_DO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_timer_value_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_counter_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_fieldbus_srw_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_SI_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_SI_sim_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_SO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_MO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_VO_array(HROBOT s, int *indexes, int *values, int len);
HRSDK_API int __stdcall set_RO_array(HROBOT s, int *indexes, int *values, int len);

HRSDK_API int __stdcall get_hrss_sdkver(HROBOT s, int &large_ver, int &small_ver, int &revision);
HRSDK_API int __stdcall get_hrsdk_sdkver(int &large_ver, int &small_ver, int &revision);

// Robot Data
HRSDK_API int __stdcall get_robot_data(HROBOT s, int *sys_info, int *port_info, double *axis_info);
HRSDK_API int __stdcall get_robot_dh(HROBOT s, int type, double dh_value[][6]);
HRSDK_API int __stdcall get_gear_ratio(HROBOT s, double gear_ratio[6]);

// HMI
HRSDK_API int __stdcall upload_hmi(HROBOT s, char *source_path, char *dest_path);
HRSDK_API int __stdcall close_hmi(HROBOT s);

// Weld
HRSDK_API int __stdcall get_weld_setting_parameter(HROBOT s, WeldSystemParameter *param);
HRSDK_API int __stdcall set_weld_setting_parameter(HROBOT s, WeldSystemParameter param);
HRSDK_API int __stdcall get_procedure_parameter(HROBOT s, int proc_id, WeldProcedureParameter *param);
HRSDK_API int __stdcall set_procedure_parameter(HROBOT s, int proc_id, WeldProcedureParameter param);
HRSDK_API int __stdcall get_procedure_comment(HROBOT s, int procedure_index, wchar_t *comment);
HRSDK_API int __stdcall set_procedure_comment(HROBOT s, int procedure_index, wchar_t *comment);
HRSDK_API int __stdcall get_schedule_parameter(HROBOT s, int proc_id, int sch_id, WeldScheduleParameter *param);
HRSDK_API int __stdcall set_schedule_parameter(HROBOT s, int proc_id, int sch_id, WeldScheduleParameter param);
HRSDK_API int __stdcall get_weave_parameter(HROBOT s, ROBOT_WEAVING_CONDITION *param);
HRSDK_API int __stdcall set_weave_parameter(HROBOT s, ROBOT_WEAVING_CONDITION param);
HRSDK_API int __stdcall get_weave_schedule_parameter(HROBOT s, int sch_id, ROBOT_WEAVING_CONDITION *param);
HRSDK_API int __stdcall set_weave_schedule_parameter(HROBOT s, int sch_id, ROBOT_WEAVING_CONDITION param);

extern uint16_t ts;

#ifdef __cplusplus
}
#endif
#endif  // HRSDK_HRSDK_H_
