/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sdlog2.c
 *
 * Simple SD logger for flight data. Buffers new sensor values and
 * does the heavy SD I/O in a low-priority worker thread.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <px4_posix.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <systemlib/err.h>
#include <unistd.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <time.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/system_power.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/encoders.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/time_offset.h>
#include <uORB/topics/mc_att_ctrl_status.h>
#include <uORB/topics/ekf2_innovations.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/ekf2_replay.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/commander_state.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/git_version.h>
#include <systemlib/printload.h>
#include <systemlib/mavlink_log.h>
#include <version/version.h>

/****************************************************************************/
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
/****************************************************************************/

#include "logbuffer.h"
#include "sdlog2_format.h"
#include "sdlog2_messages.h"

#include <drivers/drv_led.h>
extern void led_toggle(int led);
extern void led_on(int led);
extern void led_off(int led);

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu9250_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu9250_gyro"
#define MPU_DEVICE_PATH_MAG		"/dev/mpu9250_mag"

#define DEBUG_PRN
#define PX4_EPOCH_SECS 1234567890L

#define LOGBUFFER_WRITE_AND_COUNT(_msg) pthread_mutex_lock(&logbuffer_mutex); \
	if (logbuffer_write(&lb, &log_msg, LOG_PACKET_SIZE(_msg))) { \
		log_msgs_written++; \
	} else { \
		log_msgs_skipped++; \
	} \
	pthread_mutex_unlock(&logbuffer_mutex);

#define SDLOG_MIN(X,Y) ((X) < (Y) ? (X) : (Y))

static bool main_thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;			/**< Deamon status flag */
static int deamon_task;						/**< Handle of deamon task / thread */
static bool logwriter_should_exit = false;	/**< Logwriter thread exit flag */
static const unsigned MAX_NO_LOGFOLDER = 999;	/**< Maximum number of log dirs */
static const unsigned MAX_NO_LOGFILE = 999;		/**< Maximum number of log files */
static const int LOG_BUFFER_SIZE_DEFAULT = 8192;
static const int MAX_WRITE_CHUNK = 512;
static const int MIN_BYTES_TO_WRITE = 512;

static bool _extended_logging = false;
static bool _gpstime_only = false;
static int32_t _utc_offset = 0;

#ifndef __PX4_POSIX_EAGLE
#define MOUNTPOINT PX4_ROOTFSDIR"/fs/microsd"
#else
#define MOUNTPOINT "/root"
#endif
static const char *mountpoint = MOUNTPOINT;
static const char *log_root = MOUNTPOINT "/log";
static orb_advert_t mavlink_log_pub = NULL;
struct logbuffer_s lb;

/* mutex / condition to synchronize threads */
static pthread_mutex_t logbuffer_mutex;
static pthread_cond_t logbuffer_cond;

#define LOG_BASE_PATH_LEN	64

static char log_dir[LOG_BASE_PATH_LEN];

/* statistics counters */
static uint64_t start_time = 0;
static unsigned long log_bytes_written = 0;
static unsigned long last_checked_bytes_written = 0;
static unsigned long log_msgs_written = 0;
static unsigned long log_msgs_skipped = 0;

/* GPS time, used for log files naming */
static uint64_t gps_time_sec = 0;
static bool has_gps_3d_fix = false;

/* current state of logging */
static bool logging_enabled = false;
/* use date/time for naming directories and files (-t option) */
static bool log_name_timestamp = false;

/* helper flag to track system state changes */
static bool flag_system_armed = false;

/* flag if warning about MicroSD card being almost full has already been sent */
static bool space_warning_sent = false;

static pthread_t logwriter_pthread = 0;
static pthread_attr_t logwriter_attr;

static perf_counter_t perf_write;

static int possender_thread(int argc, char *argv[]);
/**
 * Log buffer writing thread. Open and close file here.
 */
static void *logwriter_thread(void *arg);

/**
 * SD log management function.
 */
__EXPORT int sdlog2_main(int argc, char *argv[]);

static bool copy_if_updated(orb_id_t topic, int *handle, void *buffer);
static bool copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle, void *buffer);

/**
 * Mainloop of sd log deamon.
 */
int sdlog2_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void sdlog2_usage(const char *reason);

/**
 * Print the current status.
 */
static void sdlog2_status(void);

/**
 * Start logging: create new file and start log writer thread.
 */
static void sdlog2_start_log(void);

/**
 * Stop logging: stop log writer thread and close log file.
 */
static void sdlog2_stop_log(void);

/**
 * Write a header to log file: list of message formats.
 */
static int write_formats(int fd);

/**
 * Write version message to log file.
 */
//static int write_version(int fd);

/**
 * Write parameters to log file.
 */
//static int write_parameters(int fd);

static bool file_exist(const char *filename);

/**
 * Check if there is still free space available
 */
static int check_free_space(void);

//static void handle_command(struct vehicle_command_s *cmd);

//static void handle_status(struct vehicle_status_s *cmd);

/**
 * Create dir for current logging session. Store dir name in 'log_dir'.
 */
static int create_log_dir(void);

/**
 * Get the time struct from the currently preferred time source
 */
static bool get_log_time_tt(struct tm *tt, bool boot_time);

/**
 * Select first free log file name and open it.
 */
static int open_log_file(void);

static int open_perf_file(const char* str);

static void
sdlog2_usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	PX4_WARN("usage: sdlog2 {start|stop|status|on|off} [-r <log rate>] [-b <buffer size>] -e -a -t -x\n"
		 "\t-r\tLog rate in Hz, 0 means unlimited rate\n"
		 "\t-b\tLog buffer size in KiB, default is 8\n"
		 "\t-e\tEnable logging by default (if not, can be started by command)\n"
		 "\t-a\tLog only when armed (can be still overriden by command)\n"
		 "\t-t\tUse date/time for naming log directories and files\n"
		 "\t-x\tExtended logging");
}

/**
 * The logger deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int sdlog2_main(int argc, char *argv[])
{
	if (argc < 2) {
		sdlog2_usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			PX4_WARN("already running");
			/* this is not an error */
			return 0;
		}

		// get sdlog priority boost parameter. This can be used to avoid message drops
		// in the log file. However, it considered to be used only for developers.
		param_t prio_boost_handle = param_find("SDLOG_PRIO_BOOST");
		int prio_boost = 0;
		param_get(prio_boost_handle, &prio_boost);
		int task_priority = SCHED_PRIORITY_DEFAULT - 30;

		switch(prio_boost) {
			case 1:
				task_priority = SCHED_PRIORITY_DEFAULT;
				break;
			case 2:
				task_priority = SCHED_PRIORITY_DEFAULT + (SCHED_PRIORITY_MAX - SCHED_PRIORITY_DEFAULT) / 2;
				break;
			case 3:
				task_priority = SCHED_PRIORITY_MAX;
				break;
			default:
				// use default priority already set above
				break;
		}

		main_thread_should_exit = false;
		
		//FIXME
		//task_priority = SCHED_PRIORITY_DEFAULT;
		task_priority = 150;
		deamon_task = px4_task_spawn_cmd("sdlog2",
						 SCHED_DEFAULT,
						 task_priority,
						 3400,
						 sdlog2_thread_main,
						 (char * const *)argv);

		//FIXME
		px4_task_spawn_cmd("possender",
						 SCHED_DEFAULT,
						 90,
						 3400,
						 possender_thread,
						 (char * const *)argv);
						 
		/* wait for the task to launch */
		unsigned const max_wait_us = 1000000;
		unsigned const max_wait_steps = 2000;

		unsigned i;
		for (i = 0; i < max_wait_steps; i++) {
			usleep(max_wait_us / max_wait_steps);
			if (thread_running) {
				break;
			}
		}

		return !(i < max_wait_steps);
	}

	if (!strcmp(argv[1], "stop")) {
		if (!thread_running) {
			PX4_WARN("not started");
		}

		main_thread_should_exit = true;
		return 0;
	}

	if (!thread_running) {
		PX4_WARN("not started\n");
		return 1;
	}

	if (!strcmp(argv[1], "status")) {
		sdlog2_status();
		return 0;
	}

	if (!strcmp(argv[1], "on")) {
		struct vehicle_command_s cmd;
		cmd.command = VEHICLE_CMD_PREFLIGHT_STORAGE;
		cmd.param1 = -1;
		cmd.param2 = -1;
		cmd.param3 = 1;
		orb_advertise(ORB_ID(vehicle_command), &cmd);
		return 0;
	}

	if (!strcmp(argv[1], "off")) {
		struct vehicle_command_s cmd;
		cmd.command = VEHICLE_CMD_PREFLIGHT_STORAGE;
		cmd.param1 = -1;
		cmd.param2 = -1;
		cmd.param3 = 2;
		orb_advertise(ORB_ID(vehicle_command), &cmd);
		return 0;
	}

	sdlog2_usage("unrecognized command");
	return 1;
}

bool get_log_time_tt(struct tm *tt, bool boot_time) {
	struct timespec ts;
	px4_clock_gettime(CLOCK_REALTIME, &ts);
	/* use RTC time for log file naming, e.g. /fs/microsd/2014-01-19/19_37_52.px4log */
	time_t utc_time_sec;

	if (_gpstime_only && has_gps_3d_fix) {
		utc_time_sec = gps_time_sec;
	} else {
		utc_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);
	}

	if (utc_time_sec > PX4_EPOCH_SECS) {
		/* strip the time elapsed since boot */
		if (boot_time) {
			utc_time_sec -= hrt_absolute_time() / 1e6;
		}

		/* apply utc offset (min, not hour) */
		utc_time_sec += _utc_offset*60;

		struct tm *ttp = gmtime_r(&utc_time_sec, tt);
		return (ttp != NULL);
	} else {
		return false;
	}
}

int create_log_dir()
{
	/* create dir on sdcard if needed */
	uint16_t dir_number = 1; // start with dir sess001
	int mkdir_ret;

	struct tm tt;
	bool time_ok = get_log_time_tt(&tt, true);

	if (log_name_timestamp && time_ok) {
		int n = snprintf(log_dir, sizeof(log_dir), "%s/", log_root);
		strftime(log_dir + n, sizeof(log_dir) - n, "%Y-%m-%d", &tt);
		mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

		if ((mkdir_ret != OK) && (errno != EEXIST)) {
			warn("failed creating new dir: %s", log_dir);
			return -1;
		}

	} else {
		/* look for the next dir that does not exist */
		while (dir_number <= MAX_NO_LOGFOLDER) {
			/* format log dir: e.g. /fs/microsd/sess001 */
			sprintf(log_dir, "%s/sess%03u", log_root, dir_number);
			mkdir_ret = mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

			if (mkdir_ret == 0) {
				break;

			} else if (errno != EEXIST) {
				warn("failed creating new dir: %s", log_dir);
				return -1;
			}

			/* dir exists already */
			dir_number++;
			continue;
		}

		if (dir_number >= MAX_NO_LOGFOLDER) {
			/* we should not end up here, either we have more than MAX_NO_LOGFOLDER on the SD card, or another problem */
			PX4_WARN("all %d possible dirs exist already", MAX_NO_LOGFOLDER);
			return -1;
		}
	}

	/* print logging path, important to find log file later */
	mavlink_and_console_log_info(&mavlink_log_pub, "[blackbox] %s", log_dir);

	return 0;
}

int open_log_file()
{
	/* string to hold the path to the log */
	char log_file_name[64] = "";
	char log_file_path[sizeof(log_file_name) + LOG_BASE_PATH_LEN] = "";

	struct tm tt;
	bool time_ok = get_log_time_tt(&tt, false);

	/* start logging if we have a valid time and the time is not in the past */
	if (log_name_timestamp && time_ok) {
		strftime(log_file_name, sizeof(log_file_name), "%H_%M_%S.px4log", &tt);
		snprintf(log_file_path, sizeof(log_file_path), "%s/%s", log_dir, log_file_name);

	} else {
		uint16_t file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.px4log */
			snprintf(log_file_name, sizeof(log_file_name), "log%03u.px4log", file_number);
			snprintf(log_file_path, sizeof(log_file_path), "%s/%s", log_dir, log_file_name);

			if (!file_exist(log_file_path)) {
				break;
			}

			file_number++;
		}

		if (file_number > MAX_NO_LOGFILE) {
			/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
			mavlink_and_console_log_critical(&mavlink_log_pub, "[blackbox] ERR: max files %d", MAX_NO_LOGFILE);
			return -1;
		}
	}

#ifdef __PX4_NUTTX
	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC);
#else
	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC, PX4_O_MODE_666);
#endif

	if (fd < 0) {
		mavlink_and_console_log_critical(&mavlink_log_pub, "[blackbox] failed: %s", log_file_name);

	} else {
		mavlink_and_console_log_info(&mavlink_log_pub, "[blackbox] recording: %s", log_file_name);
	}

	return fd;
}

int open_perf_file(const char* str)
{
	/* string to hold the path to the log */
	char log_file_name[64] = "";
	char log_file_path[sizeof(log_file_name) + LOG_BASE_PATH_LEN] = "";

	struct tm tt;
	bool time_ok = get_log_time_tt(&tt, false);

	if (log_name_timestamp && time_ok) {
		strftime(log_file_name, sizeof(log_file_name), "perf%H_%M_%S.txt", &tt);
		snprintf(log_file_path, sizeof(log_file_path), "%s/%s_%s", log_dir, str, log_file_name);

	} else {
		unsigned file_number = 1; // start with file log001

		/* look for the next file that does not exist */
		while (file_number <= MAX_NO_LOGFILE) {
			/* format log file path: e.g. /fs/microsd/sess001/log001.txt */
			snprintf(log_file_name, sizeof(log_file_name), "perf%03u.txt", file_number);
			snprintf(log_file_path, sizeof(log_file_path), "%s/%s_%s", log_dir, str, log_file_name);

			if (!file_exist(log_file_path)) {
				break;
			}

			file_number++;
		}

		if (file_number > MAX_NO_LOGFILE) {
			/* we should not end up here, either we have more than MAX_NO_LOGFILE on the SD card, or another problem */
			mavlink_and_console_log_critical(&mavlink_log_pub, "[blackbox] ERR: max files %d", MAX_NO_LOGFILE);
			return -1;
		}
	}

#ifdef __PX4_NUTTX
	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC);
#else
	int fd = open(log_file_path, O_CREAT | O_WRONLY | O_DSYNC, 0666);
#endif

	if (fd < 0) {
		mavlink_and_console_log_critical(&mavlink_log_pub, "[blackbox] failed: %s", log_file_name);

	}

	return fd;
}

static int possender_thread(int argc, char *argv[])
{
	int pos_sub;
	px4_pollfd_struct_t fds;

	pos_sub 	= orb_subscribe(ORB_ID(vehicle_gps_position));
	fds.fd 		= pos_sub;
	fds.events 	= POLLIN;

	struct vehicle_gps_position_s buf_gps_pos;
	memset(&buf_gps_pos, 0, sizeof(buf_gps_pos));
	
	while(true)
	{
		int ret = px4_poll(&fds, 1, 1000);
		if(ret > 0)
		{
			orb_copy(ORB_ID(vehicle_gps_position), pos_sub, &buf_gps_pos);
			printf("\tGPS Pos is: lat:%X\tlon:%X\t num of sat:%X\n", buf_gps_pos.lat, buf_gps_pos.lon, buf_gps_pos.satellites_used);
		}
		
		usleep(500000);
		//printf("possender_thread\n");
	}

	return 0;
}

static void *logwriter_thread(void *arg)
{
	/* set name */
	px4_prctl(PR_SET_NAME, "sdlog2_writer", 0);

	int log_fd = open_log_file();

	if (log_fd < 0) {
		return NULL;
	}

	struct logbuffer_s *logbuf = (struct logbuffer_s *)arg;

	/* write log messages formats, version and parameters */
	log_bytes_written += write_formats(log_fd);

	//log_bytes_written += write_version(log_fd);

	//log_bytes_written += write_parameters(log_fd);

	fsync(log_fd);

	int poll_count = 0;

	void *read_ptr;

	int n = 0;

	bool should_wait = false;

	bool is_part = false;

	while (true) {
		/* make sure threads are synchronized */
		pthread_mutex_lock(&logbuffer_mutex);

		/* update read pointer if needed */
		if (n > 0) {
			logbuffer_mark_read(&lb, n);
		}

		/* only wait if no data is available to process */
		if (should_wait && !logwriter_should_exit) {
			/* blocking wait for new data at this line */
			pthread_cond_wait(&logbuffer_cond, &logbuffer_mutex);
		}

		/* only get pointer to thread-safe data, do heavy I/O a few lines down */
		int available = logbuffer_get_ptr(logbuf, &read_ptr, &is_part);

		/* continue */
		pthread_mutex_unlock(&logbuffer_mutex);

		if (available > 0) {

			/* do heavy IO here */
			if (available > MAX_WRITE_CHUNK) {
				n = MAX_WRITE_CHUNK;

			} else {
				n = available;
			}

			perf_begin(perf_write);
			n = write(log_fd, read_ptr, n);
			perf_end(perf_write);

			should_wait = (n == available) && !is_part;

			if (n < 0) {
				main_thread_should_exit = true;
				warn("error writing log file");
				break;
			}

			if (n > 0) {
				log_bytes_written += n;
			}

		} else {
			n = 0;

			/* exit only with empty buffer */
			if (main_thread_should_exit || logwriter_should_exit) {
				break;
			}

			should_wait = true;
		}

		if (++poll_count == 10) {
			fsync(log_fd);
			poll_count = 0;

		}

		if (log_bytes_written - last_checked_bytes_written > 20*1024*1024) {
			/* check if space is available, if not stop everything */
			if (check_free_space() != OK) {
				logwriter_should_exit = true;
				main_thread_should_exit = true;
			}
			last_checked_bytes_written = log_bytes_written;
		}
	}

	fsync(log_fd);
	close(log_fd);

	return NULL;
}

void sdlog2_start_log()
{
	if (logging_enabled) {
		return;
	}

	/* create log dir if needed */
	if (create_log_dir() != 0) {
		mavlink_and_console_log_critical(&mavlink_log_pub, "[blackbox] error creating log dir");
		return;
	}

	/* initialize statistics counter */
	log_bytes_written = 0;
	start_time = hrt_absolute_time();
	log_msgs_written = 0;
	log_msgs_skipped = 0;

	/* initialize log buffer emptying thread */
	pthread_attr_init(&logwriter_attr);

#ifndef __PX4_POSIX_EAGLE
	struct sched_param param;
	(void)pthread_attr_getschedparam(&logwriter_attr, &param);
	/* low priority, as this is expensive disk I/O. */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 5;
	if (pthread_attr_setschedparam(&logwriter_attr, &param)) {
		PX4_WARN("sdlog2: failed setting sched params");
	}
#endif

	pthread_attr_setstacksize(&logwriter_attr, 2048);

	logwriter_should_exit = false;

	/* allocate write performance counter */
	perf_write = perf_alloc(PC_ELAPSED, "sd write");

	/* start log buffer emptying thread */
	if (0 != pthread_create(&logwriter_pthread, &logwriter_attr, logwriter_thread, &lb)) {
		PX4_WARN("error creating logwriter thread");
	}

	// if (0 != pthread_create(&possender_thread, &logwriter_attr, logwriter_thread, &lb)) 
	// {
		// PX4_WARN("error creating logwriter thread");
	// }

	/* write all performance counters */
	hrt_abstime curr_time = hrt_absolute_time();
	struct print_load_s load;
	int perf_fd = open_perf_file("preflight");
	init_print_load_s(curr_time, &load);
	print_load(curr_time, perf_fd, &load);
	dprintf(perf_fd, "PERFORMANCE COUNTERS PRE-FLIGHT\n\n");
	perf_print_all(perf_fd);
	dprintf(perf_fd, "\nLOAD PRE-FLIGHT\n\n");
	usleep(500 * 1000);
	print_load(hrt_absolute_time(), perf_fd, &load);
	close(perf_fd);

	/* reset performance counters to get in-flight min and max values in post flight log */
	perf_reset_all();

	logging_enabled = true;
}

void sdlog2_stop_log()
{
	if (!logging_enabled) {
		return;
	}

	/* disabling the logging will trigger the skipped count to increase,
	 * so we take a local copy before interrupting the disk I/O.
	 */
	unsigned long skipped_count = log_msgs_skipped;

	logging_enabled = false;

	/* wake up write thread one last time */
	pthread_mutex_lock(&logbuffer_mutex);
	logwriter_should_exit = true;
	pthread_cond_signal(&logbuffer_cond);
	/* unlock, now the writer thread may return */
	pthread_mutex_unlock(&logbuffer_mutex);

	/* wait for write thread to return */
	int ret;

	if ((ret = pthread_join(logwriter_pthread, NULL)) != 0) {
		PX4_WARN("error joining logwriter thread: %i", ret);
	}

	logwriter_pthread = 0;
	pthread_attr_destroy(&logwriter_attr);

	/* write all performance counters */
	int perf_fd = open_perf_file("postflight");
	hrt_abstime curr_time = hrt_absolute_time();
	dprintf(perf_fd, "PERFORMANCE COUNTERS POST-FLIGHT\n\n");
	perf_print_all(perf_fd);
	struct print_load_s load;
	dprintf(perf_fd, "\nLOAD POST-FLIGHT\n\n");
	init_print_load_s(curr_time, &load);
	print_load(curr_time, perf_fd, &load);
	sleep(1);
	print_load(hrt_absolute_time(), perf_fd, &load);
	close(perf_fd);

	/* free log writer performance counter */
	perf_free(perf_write);

	/* reset the logbuffer */
	logbuffer_reset(&lb);

	mavlink_and_console_log_info(&mavlink_log_pub, "[blackbox] stopped (%lu drops)", skipped_count);

	sdlog2_status();
}

int write_formats(int fd)
{
	/* construct message format packet */
	struct {
		LOG_PACKET_HEADER;
		struct log_format_s body;
	} log_msg_format = {
		LOG_PACKET_HEADER_INIT(LOG_FORMAT_MSG),
	};

	int written = 0;

	/* fill message format packet for each format and write it */
	for (unsigned i = 0; i < log_formats_num; i++) {
		log_msg_format.body = log_formats[i];
		written += write(fd, &log_msg_format, sizeof(log_msg_format));
	}

	return written;
}

#if 0
int write_version(int fd)
{
	/* construct version message */
	struct {
		LOG_PACKET_HEADER;
		struct log_VER_s body;
	} log_msg_VER = {
		LOG_PACKET_HEADER_INIT(LOG_VER_MSG),
	};

	/* fill version message and write it */
	strncpy(log_msg_VER.body.fw_git, px4_git_version, sizeof(log_msg_VER.body.fw_git));
	strncpy(log_msg_VER.body.arch, HW_ARCH, sizeof(log_msg_VER.body.arch));
	return write(fd, &log_msg_VER, sizeof(log_msg_VER));
}

int write_parameters(int fd)
{
	/* construct parameter message */
	struct {
		LOG_PACKET_HEADER;
		struct log_PARM_s body;
	} log_msg_PARM = {
		LOG_PACKET_HEADER_INIT(LOG_PARM_MSG),
	};

	int written = 0;
	param_t params_cnt = param_count();

	for (param_t param = 0; param < params_cnt; param++) {
		/* fill parameter message and write it */
		strncpy(log_msg_PARM.body.name, param_name(param), sizeof(log_msg_PARM.body.name));
		float value = NAN;

		switch (param_type(param)) {
		case PARAM_TYPE_INT32: {
				int32_t i;
				param_get(param, &i);
				value = i;	// cast integer to float
				break;
			}

		case PARAM_TYPE_FLOAT:
			param_get(param, &value);
			break;

		default:
			break;
		}

		log_msg_PARM.body.value = value;
		written += write(fd, &log_msg_PARM, sizeof(log_msg_PARM));
	}

	return written;
}
#endif

//#if 0
bool copy_if_updated(orb_id_t topic, int *handle, void *buffer)
{
	return copy_if_updated_multi(topic, 0, handle, buffer);
}


bool copy_if_updated_multi(orb_id_t topic, int multi_instance, int *handle, void *buffer)
{
	bool updated = false;

	if (*handle < 0) {
#if __PX4_POSIX_EAGLE
		// The orb_exists call doesn't work correctly on Snapdragon yet.
		// (No data gets sent from the QURT to the Linux side because there
		// are no subscribers. However, there won't be any subscribers, if
		// they check using orb_exists() before subscribing.)
		if (true)
#else
		if (OK == orb_exists(topic, multi_instance))
#endif

		{
			//printf("orb subscribe\n");
			*handle = orb_subscribe_multi(topic, multi_instance);
			/* copy first data */
			if (*handle >= 0) {
				orb_copy(topic, *handle, buffer);
				updated = true;
			}
		}
	} else {
		//printf("orb check\n");
		orb_check(*handle, &updated);

		if (updated) {
			orb_copy(topic, *handle, buffer);
		}
	}

	return updated;
}
//#endif

//FIXME:使用union节省空间
struct accel_report buf_acc;
struct gyro_report buf_gyr;
struct mag_report buf_mag;
int sdlog2_thread_main(int argc, char *argv[])
{
	/* default log rate: 50 Hz */
	int32_t log_rate = 50;
	int log_buffer_size = LOG_BUFFER_SIZE_DEFAULT;
	logging_enabled = false;
	/* enable logging on start (-e option) */
	bool log_on_start = false;
	/* enable logging when armed (-a option) */
	//bool log_when_armed = false;
	log_name_timestamp = false;

	flag_system_armed = false;

#ifdef __PX4_NUTTX
	/* work around some stupidity in NuttX's task_create's argv handling */
	argc -= 2;
	argv += 2;
#endif

	int ch;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	int myoptind = 1;
	const char *myoptarg = NULL;
	while ((ch = px4_getopt(argc, argv, "r:b:eatx", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(myoptarg, NULL, 10);

				if (r <= 0) {
					r = 1;
				}

				log_rate = r;
			}
			break;

		case 'b': {
				unsigned long s = strtoul(myoptarg, NULL, 10);

				if (s < 1) {
					s = 1;
				}

				log_buffer_size = 1024 * s;
			}
			break;

		case 'e':
			log_on_start = true;
			break;

		case 'a':
			//log_when_armed = true;
			break;

		case 't':
			log_name_timestamp = true;
			break;

		case 'x':
			_extended_logging = true;
			break;

		case '?':
			if (optopt == 'c') {
				PX4_WARN("option -%c requires an argument", optopt);

			} else if (isprint(optopt)) {
				PX4_WARN("unknown option `-%c'", optopt);

			} else {
				PX4_WARN("unknown option character `\\x%x'", optopt);
			}
			err_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		sdlog2_usage(NULL);
	}

	gps_time_sec = 0;

	/* interpret logging params */
	int32_t param_log_rate = -1;
	param_t log_rate_ph = param_find("SDLOG_RATE");

	if (log_rate_ph != PARAM_INVALID) {
		param_get(log_rate_ph, &param_log_rate);

		if (param_log_rate > 0) {

			/* we can't do more than ~ 500 Hz, even with a massive buffer */
			if (param_log_rate > 250) {
				param_log_rate = 250;
			}

		} else if (param_log_rate == 0) {
			/* we need at minimum 10 Hz to be able to see anything */
			param_log_rate = 10;
		}
	}

	// if parameter was provided use it, if not use command line argument
	log_rate = param_log_rate > -1 ? param_log_rate : log_rate;

	param_t log_ext_ph = param_find("SDLOG_EXT");

	if (log_ext_ph != PARAM_INVALID) {

		int32_t param_log_extended;
		param_get(log_ext_ph, &param_log_extended);

		if (param_log_extended > 0) {
			_extended_logging = true;
		} else if (param_log_extended == 0) {
			_extended_logging = false;
		}
		/* any other value means to ignore the parameter, so no else case */

	}

	param_t log_gpstime_ph = param_find("SDLOG_GPSTIME");

	if (log_gpstime_ph != PARAM_INVALID) {

		int32_t param_log_gpstime;
		param_get(log_gpstime_ph, &param_log_gpstime);

		if (param_log_gpstime > 0) {
			_gpstime_only = true;
		} else if (param_log_gpstime == 0) {
			_gpstime_only = false;
		}
		/* any other value means to ignore the parameter, so no else case */

	}

	param_t log_utc_offset = param_find("SDLOG_UTC_OFFSET");

	if ( log_utc_offset != PARAM_INVALID ) {
	    int32_t param_utc_offset;
	    param_get(log_utc_offset, &param_utc_offset);
	    _utc_offset = param_utc_offset;
	}

	if (check_free_space() != OK) {
		PX4_WARN("ERR: MicroSD almost full");
		return 1;
	}


	/* create log root dir */
	int mkdir_ret = mkdir(log_root, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret != 0 && errno != EEXIST) {
		warn("ERR: failed creating log dir: %s", log_root);
		return 1;
	}

	/* initialize log buffer with specified size */
	PX4_WARN("log buffer size: %i bytes", log_buffer_size);

	if (OK != logbuffer_init(&lb, log_buffer_size)) {
		PX4_WARN("can't allocate log buffer, exiting");
		return 1;
	}

	struct vehicle_status_s buf_status;
	memset(&buf_status, 0, sizeof(buf_status));

	struct vehicle_gps_position_s buf_gps_pos;
	memset(&buf_gps_pos, 0, sizeof(buf_gps_pos));

	struct vehicle_command_s buf_cmd;
	memset(&buf_cmd, 0, sizeof(buf_cmd));

	struct commander_state_s buf_commander_state;
	memset(&buf_commander_state, 0, sizeof(buf_commander_state));

	// check if we are gathering data for a replay log for ekf2
	// is yes then disable logging of some topics to avoid dropouts
	param_t replay_handle = param_find("EKF2_REC_RPL");
	int32_t tmp = 0;
	param_get(replay_handle, &tmp);
	bool record_replay_log = (bool)tmp;

	/* warning! using union here to save memory, elements should be used separately! */
//	union {
//		struct vehicle_command_s cmd;
//		struct sensor_combined_s sensor;
//	} buf;

//	memset(&buf, 0, sizeof(buf));

	/* log message buffer: header + body */
#pragma pack(push, 1)
	struct {
		LOG_PACKET_HEADER;
		union {
			struct log_TIME_s log_TIME;
			struct log_IMU_s  log_IMU;
			struct log_GPS_s  log_GPS;
		} body;
	} log_msg = {
		LOG_PACKET_HEADER_INIT(0)
	};
#pragma pack(pop)
	memset(&log_msg.body, 0, sizeof(log_msg.body));

//FIXME
	struct {
		int gps_pos_sub;
	} subs;
	
	int gps_sub = -1;
	bool gps_pos_updated = false;

	subs.gps_pos_sub = -1;

	/* initialize thread synchronization */
	pthread_mutex_init(&logbuffer_mutex, NULL);
	pthread_cond_init(&logbuffer_cond, NULL);

	/* enable logging on start if needed */
	if (log_on_start) {
		/* check GPS topic to get GPS time */
		if (log_name_timestamp) {
			if (!orb_copy(ORB_ID(vehicle_gps_position), subs.gps_pos_sub, &buf_gps_pos)) {
				gps_time_sec = buf_gps_pos.time_utc_usec / 1e6;
			}
		}

		sdlog2_start_log();
	}

	/* running, report */
	thread_running = true;

	// wakeup source
	//px4_pollfd_struct_t fds[1];

	int poll_counter = 0;

	int poll_to_logging_factor = 1;

	if (record_replay_log) {

	} else {
//		subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
//		fds[0].fd = subs.sensor_sub;
//		fds[0].events = POLLIN;
		// TODO Remove hardcoded rate!
		poll_to_logging_factor = 250 / (log_rate < 1 ? 1 : log_rate);
	}

	if (poll_to_logging_factor < 1) {
		poll_to_logging_factor = 1;
	}

	int fd_acc = -1;
	int fd_gyr = -1;
	int fd_mag = -1;
	
//	struct accel_report buf_acc;
//	struct gyro_report buf_gyr;
//	struct mag_report buf_mag;
	int	ret;

	//fd_acc = px4_open(MPU_DEVICE_PATH_ACCEL, O_RDONLY);
	for(int i = 0; i < 3; i++)
	{
		usleep(500000);

		if (fd_acc < 0) {
			fd_acc = px4_open(ACCEL0_DEVICE_PATH, O_RDONLY);

			if(fd_acc < 0)
				printf("ACCEL: open fail\n");
		}

	//fd_gyr = px4_open(MPU_DEVICE_PATH_GYRO, O_RDONLY);
		if (fd_gyr < 0) {
			fd_gyr = px4_open(GYRO0_DEVICE_PATH, O_RDONLY);

			if (fd_gyr < 0)
				printf("GYRO: open fail\n");
//		return ERROR;
		}
	
	//fd_mag = px4_open(MPU_DEVICE_PATH_MAG, O_RDONLY);
		if (fd_mag < 0) {
			fd_mag = px4_open(MAG0_DEVICE_PATH, O_RDONLY);

			if (fd_mag < 0)
				printf("MAG: open fail\n");
//		return ERROR;
		}

		if(fd_acc >= 0 && fd_gyr >= 0 && fd_mag >= 0)
			break;
	}
	
	int size_ba = sizeof(buf_acc);
	int size_bg = sizeof(buf_gyr);
	int size_bm = sizeof(buf_mag);

	//printf("log packet size is %d\n", LOG_PACKET_SIZE(IMU));
	//printf("size of log_msg.body is %d\n", sizeof(log_msg.body));
//	printf("size of float is %d\n", sizeof(float));

//	uint64_t t0, t1, t2, t3, t4, t5, t6, t7, t8;
//	t0 = t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = 0;
//	uint64_t t0 = 0;

	int c = 0;
	bool gps_ok =  false;
	while (!main_thread_should_exit) 
	{
		/* wait at least 100ms, sensor should have data after no more than 20ms */
//t0 = hrt_absolute_time();
//printf("Time: %lld \n", t0);
		usleep(8970);
		//usleep(500);

		ret = px4_read(fd_mag, &buf_mag, size_bm);
/*		if (ret != size_bm) 
		{

#undef DEBUG_PRN
#ifdef DEBUG_PRN
			printf("\tMAG: read fail (%d)\n", ret);
#endif
			//continue;
			for(int i = 0; i < 3; i++)
			{
				usleep(500);
				ret = px4_read(fd_mag, &buf_mag, size_bm);
				if (ret == size_bm)
					break;
			}

			if(ret != size_bm)
			{
#ifdef DEBUG_PRN
				printf("\tMAG: read fail (%d), 3 times\n", ret);
#endif
				continue;
			}
		}
*/
		ret = px4_read(fd_acc, &buf_acc, size_ba);
//t1 = hrt_absolute_time();
		if (ret != size_ba) {
#undef DEBUG_PRN
#ifdef DEBUG_PRN
			printf("\tACCEL: read1 fail (%d)\n", ret);
#endif
			continue;
		}
//printf("acc DT:%lld  x:%f  y:%f  z:%f  x_integral:%f  y:%f  z:%f  \n", 
//	buf_acc.integral_dt, (double)buf_acc.x, (double)buf_acc.y, (double)buf_acc.z);
		
		ret = px4_read(fd_gyr, &buf_gyr, size_bg);
		if (ret != size_bg) {
#ifdef DEBUG_PRN
			printf("\tGYRO: read fail (%d)\n", ret);
#endif
			continue;
		}


//t2 = hrt_absolute_time();
//printf("\tTime is: %lld\t%lld\t%lld\n", t3, t1, t0);
		if (!logging_enabled) {
//FIXME
//			continue;
		}
		//所有的传感器数据读正确，拷贝数据
		log_msg.msg_type = LOG_IMU_MSG;
		log_msg.body.log_IMU.acc_x    = buf_acc.x;
		log_msg.body.log_IMU.acc_y    = buf_acc.y;
		log_msg.body.log_IMU.acc_z    = buf_acc.z;
		log_msg.body.log_IMU.gyr_x   = buf_gyr.x;
		log_msg.body.log_IMU.gyr_y   = buf_gyr.y;
		log_msg.body.log_IMU.gyr_z   = buf_gyr.z;
		log_msg.body.log_IMU.mag_x    = buf_mag.x;
		log_msg.body.log_IMU.mag_y    = buf_mag.y;
		log_msg.body.log_IMU.mag_z    = buf_mag.z;
		
		/*log_msg.body.log_IMU.acc_scaling = buf_acc.scaling;
		log_msg.body.log_IMU.gyr_scaling = buf_gyr.scaling;
		log_msg.body.log_IMU.mag_scaling = buf_mag.scaling;
		
		log_msg.body.log_IMU.acc_dt = buf_acc.integral_dt;
		log_msg.body.log_IMU.gyr_dt = buf_gyr.integral_dt;*/
		log_msg.body.log_IMU.acc_time = buf_acc.timestamp;
		log_msg.body.log_IMU.gyr_time = buf_gyr.timestamp;
		log_msg.body.log_IMU.mag_time = buf_mag.timestamp;
		
		/*log_msg.body.log_IMU.acc_x_raw = buf_acc.x_raw;
		log_msg.body.log_IMU.acc_y_raw = buf_acc.y_raw;
		log_msg.body.log_IMU.acc_Z_raw = buf_acc.z_raw;
		log_msg.body.log_IMU.gyr_x_raw = buf_gyr.x_raw;
		log_msg.body.log_IMU.gyr_y_raw = buf_gyr.y_raw;
		log_msg.body.log_IMU.gyr_Z_raw = buf_gyr.z_raw;
		log_msg.body.log_IMU.mag_x_raw = buf_mag.x_raw;
		log_msg.body.log_IMU.mag_y_raw = buf_mag.y_raw;
		log_msg.body.log_IMU.mag_Z_raw = buf_mag.z_raw;*/
//		printf("\tTime stamp is: %lld\t%lld\t%lld\n", log_msg.body.log_IMU.time_acc, log_msg.body.log_IMU.time_gyr, log_msg.body.log_IMU.time_mag);
//printf("\tACCEL accel: x:%8.4f\ty:%8.4f\tz:%8.4f\n", (double)(buf_acc.x), (double)(buf_acc.y), (double)(buf_acc.z));
#undef DEBUG_PRN
//#define DEBUG_PRN
#ifdef DEBUG_PRN
//#if 0
		printf("\tACCEL accel: x:%X\ty:%X\tz:%X\n", *((int *)&(buf_acc.x)), *((int *)&(buf_acc.y)), *((int *)&(buf_acc.z)));
		printf("\tGYRO rates: x:%X\ty:%X\tz:%X\n", *((int *)&(buf_gyr.x)), *((int *)&(buf_gyr.y)), *((int *)&(buf_gyr.z)));
		printf("\tMAG values: x:%X\ty:%X\tz:%X\n", *((int *)&(buf_mag.x)), *((int *)&(buf_mag.y)), *((int *)&(buf_mag.z)));
		printf("\tTime stamp is: %llX\t%llX\t%llX\n", log_msg.body.log_IMU.acc_time, log_msg.body.log_IMU.gyr_time, log_msg.body.log_IMU.mag_time);
#endif

		LOGBUFFER_WRITE_AND_COUNT(IMU);

//t3 = hrt_absolute_time();
		gps_pos_updated = copy_if_updated(ORB_ID(vehicle_gps_position), &gps_sub, &buf_gps_pos);
		if (gps_pos_updated) 
		{
#ifdef DEBUG_PRN
			printf("\tGPS Time stamp is: %llX\tUTC Time is: %llX\n", buf_gps_pos.timestamp_position, buf_gps_pos.time_utc_usec);
			printf("\tGPS Pos is: lat:%X\tlon:%X\t num of sat:%X\n", buf_gps_pos.lat, buf_gps_pos.lon, buf_gps_pos.satellites_used);
#endif			
			if(buf_gps_pos.satellites_used > 0)
			{
				//led_off(3);
				//led_toggle(1);
				gps_ok = true;
			}
			 else
			{
				 //led_off(1);
				 gps_ok = false;
			}

			{
				log_msg.msg_type 				= LOG_GPS_MSG;
				log_msg.body.log_GPS.time_stamp = buf_gps_pos.timestamp_position;
				log_msg.body.log_GPS.time_gps	= buf_gps_pos.time_utc_usec;
				log_msg.body.log_GPS.lat		= buf_gps_pos.lat;
				log_msg.body.log_GPS.lon		= buf_gps_pos.lon;
				log_msg.body.log_GPS.alt		= buf_gps_pos.alt;

				log_msg.body.log_GPS.vel_n  	= buf_gps_pos.vel_n_m_s;
				log_msg.body.log_GPS.vel_e  	= buf_gps_pos.vel_e_m_s;
				log_msg.body.log_GPS.vel_d  	= buf_gps_pos.vel_d_m_s;
				log_msg.body.log_GPS.sats		= buf_gps_pos.satellites_used;
				LOGBUFFER_WRITE_AND_COUNT(GPS);
			}
		}
		else
		{
#ifdef DEBUG_PRN
			printf("\tGPS pos is not update\n");
#endif	
		}
//t4 = hrt_absolute_time();
		//FIXME 这一部分去掉
		if ((poll_counter + 1) % poll_to_logging_factor == 0) {
			poll_counter = 0;
		} else {
			// copy topic
			poll_counter++;
//			continue;
		}
//t5 = hrt_absolute_time();
		pthread_mutex_lock(&logbuffer_mutex);

		/* signal the other thread new data, but not yet unlock */
		if (logbuffer_count(&lb) > MIN_BYTES_TO_WRITE) {
			/* only request write if several packets can be written at once */
			pthread_cond_signal(&logbuffer_cond);
		}
//t6 = hrt_absolute_time();
		/* unlock, now the writer thread may run */
		pthread_mutex_unlock(&logbuffer_mutex);
//t7 = hrt_absolute_time();
//printf("Time: %lld  %lld  %lld  %lld  %lld  %lld  %lld  %lld  %lld\n", t0, t1, t2, t3, t4, t5, t6, t7, t8);
//#ifdef DEBUG_PRN
#if 0
//printf("GPS Time stamp is: %lld\tUTC Time is: %lld\n", buf_gps_pos.timestamp_position, buf_gps_pos.time_utc_usec);
//printf("GPS Pos is: lat:%X\tlon:%X\t num of sat:%X\n", buf_gps_pos.lat, buf_gps_pos.lon, buf_gps_pos.satellites_used);
printf("Sensors Time stamp: %lld\t%lld\t%lld\n", log_msg.body.log_IMU.acc_time, log_msg.body.log_IMU.gyr_time, log_msg.body.log_IMU.mag_time);
//printf("DT_AC: %lld  DT_GY: %lld\n", buf_acc.integral_dt, buf_gyr.integral_dt);
#endif
//t8 = hrt_absolute_time();
		c++;
		if(c % 50 == 0)
		{
			if(gps_ok)
			{
				led_off(3);
				led_toggle(1);
			}
			else
			{
				led_off(1);
				led_toggle(3);
			}

			c=0;
		}
	}

	if (logging_enabled) {
		sdlog2_stop_log();
	}

	pthread_mutex_destroy(&logbuffer_mutex);
	pthread_cond_destroy(&logbuffer_cond);

	/* free log buffer */
	logbuffer_free(&lb);

	thread_running = false;

	px4_close(fd_acc);
	px4_close(fd_gyr);
	px4_close(fd_mag);

	return 0;
}

void sdlog2_status()
{
	PX4_WARN("extended logging: %s", (_extended_logging) ? "ON" : "OFF");
	PX4_WARN("time: gps: %u seconds", (unsigned)gps_time_sec);
	if (!logging_enabled) {
		PX4_WARN("not logging");
	} else {

		float kibibytes = log_bytes_written / 1024.0f;
		float mebibytes = kibibytes / 1024.0f;
		float seconds = ((float)(hrt_absolute_time() - start_time)) / 1000000.0f;

		PX4_WARN("wrote %lu msgs, %4.2f MiB (average %5.3f KiB/s), skipped %lu msgs", log_msgs_written, (double)mebibytes, (double)(kibibytes / seconds), log_msgs_skipped);
		mavlink_log_info(&mavlink_log_pub, "[blackbox] wrote %lu msgs, skipped %lu msgs", log_msgs_written, log_msgs_skipped);
	}
}

/**
 * @return true if file exists
 */
bool file_exist(const char *filename)
{
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int check_free_space()
{
	/* use statfs to determine the number of blocks left */
	FAR struct statfs statfs_buf;
	if (statfs(mountpoint, &statfs_buf) != OK) {
		PX4_WARN("ERR: statfs");
		return PX4_ERROR;
	}

	/* use a threshold of 50 MiB */
	if (statfs_buf.f_bavail < (px4_statfs_buf_f_bavail_t)(50 * 1024 * 1024 / statfs_buf.f_bsize)) {
		mavlink_and_console_log_critical(&mavlink_log_pub, "[blackbox] no space on MicroSD: %u MiB",
			(unsigned int)(statfs_buf.f_bavail * statfs_buf.f_bsize) / (1024U * 1024U));
		/* we do not need a flag to remember that we sent this warning because we will exit anyway */
		return PX4_ERROR;

	/* use a threshold of 100 MiB to send a warning */
	} else if (!space_warning_sent && statfs_buf.f_bavail < (px4_statfs_buf_f_bavail_t)(100 * 1024 * 1024 / statfs_buf.f_bsize)) {
		mavlink_and_console_log_critical(&mavlink_log_pub, "[blackbox] space on MicroSD low: %u MiB",
			(unsigned int)(statfs_buf.f_bavail * statfs_buf.f_bsize) / (1024U * 1024U));
		/* we don't want to flood the user with warnings */
		space_warning_sent = true;
	}

	return PX4_OK;
}

#if 0
void handle_command(struct vehicle_command_s *cmd)
{
	int param;

	/* request to set different system mode */
	switch (cmd->command) {

	case VEHICLE_CMD_PREFLIGHT_STORAGE:
		param = (int)(cmd->param3 + 0.5f);

		if (param == 1)	{
			sdlog2_start_log();

		} else if (param == 2)	{
			sdlog2_stop_log();
		} else {
			// Silently ignore non-matching command values, as they could be for params.
		}

		break;

	default:
		/* silently ignore */
		break;
	}
}


void handle_status(struct vehicle_status_s *status)
{
	// TODO use flag from actuator_armed here?
	bool armed = status->arming_state == ARMING_STATE_ARMED || status->arming_state == ARMING_STATE_ARMED_ERROR;

	if (armed != flag_system_armed) {
		flag_system_armed = armed;

		if (flag_system_armed) {
			sdlog2_start_log();

		} else {
			sdlog2_stop_log();
		}
	}
}
#endif
