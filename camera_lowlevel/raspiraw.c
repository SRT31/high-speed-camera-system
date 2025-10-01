/*
Copyright (c) 2015, Raspberry Pi Foundation
Copyright (c) 2015, Dave Stevenson
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define VERSION_STRING "0.0.3"

#define _GNU_SOURCE
#include <ctype.h>
#include <fcntl.h>
#include <libgen.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#define I2C_SLAVE_FORCE 0x0706
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"

#include "interface/vcsm/user-vcsm.h"

#include "RaspiCLI.h"

#include <sys/ioctl.h>

#include "raw_header.h"
// new libraries to support RING BUFFER + WRITER THREAD
#include <pthread.h> // pthread_create, pthread_join
#include <semaphore.h> 	// sem_init, sem_wait, sem_post
#include <unistd.h>     // write, close
#include <sys/types.h>	
#include <sys/stat.h>

#define DEFAULT_I2C_DEVICE 0 // corresponds to /dev/i2c-0

#define I2C_DEVICE_NAME_LEN 13 // "/dev/i2c-XXX"+NULL

struct brcm_raw_header *brcm_header = NULL;

enum bayer_order
{
	// Carefully ordered so that an hflip is ^1,
	// and a vflip is ^2.
	BAYER_ORDER_BGGR,
	BAYER_ORDER_GBRG,
	BAYER_ORDER_GRBG,
	BAYER_ORDER_RGGB
};

struct sensor_regs
{
	uint16_t reg;
	uint16_t data;
};

struct mode_def
{
	struct sensor_regs *regs;
	int num_regs;
	int width;
	int height;
	MMAL_FOURCC_T encoding;
	enum bayer_order order;
	int native_bit_depth;
	uint8_t image_id;
	uint8_t data_lanes;
	int min_vts;
	int line_time_ns;
	uint32_t timing[5];
	uint32_t term[2];
	int black_level;

	int binning; /* Binning or skipping factor */
};

struct raspiraw_crop
{
	int hinc;
	int vinc;
	int width;
	int height;
	int left;
	int top;
};

struct sensor_def
{
	char *name;
	struct sensor_regs *common_init;
	int num_common_init;
	struct mode_def *modes;
	int num_modes;
	struct sensor_regs *stop;
	int num_stop_regs;

	uint8_t i2c_addr;   // Device I2C slave address
	int i2c_addressing; // Length of register address values
	int i2c_data_size;  // Length of register data to write

	//  Detecting the device
	int i2c_ident_length;	  // Length of I2C ID register
	uint16_t i2c_ident_reg;	  // ID register address
	uint16_t i2c_ident_value; // ID register value

	// Flip configuration
	uint16_t vflip_reg;		   // Register for VFlip
	int vflip_reg_bit;		   // Bit in that register for VFlip
	uint16_t hflip_reg;		   // Register for HFlip
	int hflip_reg_bit;		   // Bit in that register for HFlip
	int flips_dont_change_bayer_order; // Some sensors do not change the
					   // Bayer order by adjusting X/Y
					   // starts to compensate.

	uint16_t exposure_reg;
	int exposure_reg_num_bits;

	uint16_t vts_reg;
	int vts_reg_num_bits;

	uint16_t gain_reg;
	int gain_reg_num_bits;

	int (*set_crop)(const struct sensor_def *, struct mode_def *, const struct raspiraw_crop *cfg);
};

// The process first loads the cleaned up dump of the registers
// than updates the known registers to the proper values
// based on: http://www.seeedstudio.com/wiki/images/3/3c/Ov5647_full.pdf
enum operation
{
	EQUAL, // Set bit to value
	SET,   // Set bit
	CLEAR, // Clear bit
	XOR    // Xor bit
};

void modReg(struct mode_def *mode, uint16_t reg, int startBit, int endBit, int value, enum operation op);

#define NUM_ELEMENTS(a) (sizeof(a) / sizeof(a[0]))

// Supported sensors 
// Original builds multiple sensors (OV5647, IMX219, ADV7282, IMX477).
// This build narrows support to IMX219 only to reduce binary size and
// complexity; related headers and tables were removed accordingly.
#include "imx219_modes.h" 	

const struct sensor_def *sensors[] = { &imx219, NULL };  

// CLI (simplified)
// Compared to the original, many options (AWB, preview/ISP, YUV output,
// processing threads, etc.) are removed. The remaining options cover
// raw capture, timing, bit depth, I2C selection and basic ROI controls.
enum
{
	CommandHelp,
	CommandMode,
	CommandHFlip,
	CommandVFlip,
	CommandExposure,
	CommandGain,
	CommandOutput,
	CommandWriteHeader,
	CommandTimeout,
	CommandSaveRate,
	CommandBitDepth,
	CommandExposureus,
	CommandI2cBus,
	CommandRegs,
	CommandHinc,
	CommandVinc,
	CommandFps,
	CommandWidth,
	CommandHeight,
	CommandLeft,
	CommandTop,
	CommandVts,
	CommandLine,
	CommandWriteHeader0,
	CommandWriteHeaderG,
	CommandWriteTimestamps,
	CommandWriteEmpty,
};	// removed commands to simplify

static COMMAND_LIST cmdline_commands[] = {
	// clang-format off
	{ CommandHelp,            "-help",           "?",    "This help information", 0 },
	{ CommandMode,            "-mode",           "md",   "Set sensor mode <mode>", 1 },
	{ CommandHFlip,           "-hflip",          "hf",   "Set horizontal flip", 0 },
	{ CommandVFlip,           "-vflip",          "vf",   "Set vertical flip", 0 },
	{ CommandExposure,        "-ss",             "e",    "Set the sensor exposure time (not calibrated units)", 0 },
	{ CommandGain,            "-gain",           "g",    "Set the sensor gain code (not calibrated units)", 0 },
	{ CommandOutput,          "-output",         "o",    "Set the output filename", 0 },
	{ CommandWriteHeader,     "-header",         "hd",   "Write the BRCM header to the output file", 0 },
	{ CommandTimeout,         "-timeout",        "t",    "Time (in ms) before shutting down (if not specified, set to 5s)", 1 },
	{ CommandSaveRate,        "-saverate",       "sr",   "Save every Nth frame", 1 },
	{ CommandBitDepth,        "-bitdepth",       "b",    "Set output raw bit depth (8, 10, 12 or 16, if not specified, set to sensor native)", 1 },
	{ CommandExposureus,      "-expus",          "eus",  "Set the sensor exposure time in micro seconds.", -1 },
	{ CommandI2cBus,          "-i2c",            "y",    "Set the I2C bus to use.", -1 },
	{ CommandRegs,            "-regs",           "r",    "Change (current mode) regs", 0 },
	{ CommandHinc,            "-hinc",           "hi",   "Set horizontal odd/even inc reg", -1 },
	{ CommandVinc,            "-vinc",           "vi",   "Set vertical odd/even inc reg", -1 },
	{ CommandFps,             "-fps",            "f",    "Set framerate regs", -1 },
	{ CommandWidth,           "-width",          "w",    "Set current mode width", -1 },
	{ CommandHeight,          "-height",         "h",    "Set current mode height", -1 },
	{ CommandLeft,            "-left",           "lt",   "Set current mode left", -1 },
	{ CommandTop,             "-top",            "tp",   "Set current mode top", -1 },
	{ CommandWriteHeader0,    "-header0",        "hd0",  "Sets filename to write the BRCM header to", 0 },
	{ CommandWriteHeaderG,    "-headerg",        "hdg",  "Sets filename to write the .pgm header to", 0 },
	{ CommandWriteTimestamps, "-tstamps",        "ts",   "Sets filename to write timestamps to", 0 },
	{ CommandWriteEmpty,      "-empty",          "emp",  "Write empty output files", 0 },
	// clang-format on
}; // removed commands to simplify

static int cmdline_commands_size = sizeof(cmdline_commands) / sizeof(cmdline_commands[0]);

typedef struct pts_node
{
	int idx;
	int64_t pts;
	struct pts_node *nxt;
} * PTS_NODE_T;



typedef struct raspiraw_params
{
	struct raspiraw_crop crop;
	int mode;
	int hflip;
	int vflip;
	int exposure;
	int gain;
	char *output;
	int capture;
	int write_header;
	int timeout;
	int saverate;
	int bit_depth;
	int exposure_us;
	int i2c_bus;
	char *regs;
	double fps;
	char *write_header0;
	char *write_headerg;
	char *write_timestamps;
	int write_empty;
	PTS_NODE_T ptsa;
	PTS_NODE_T ptso;
} RASPIRAW_PARAMS_T; 	// removed fields to simplify
typedef struct
{
	RASPIRAW_PARAMS_T *cfg;

	MMAL_POOL_T *rawcam_pool;
	MMAL_PORT_T *rawcam_output;
	// removed fields to simplify
	
} RASPIRAW_CALLBACK_T;

/* ---------------- RING BUFFER + WRITER THREAD ----------------
   here’s the idea: 
   - The camera callback is running super fast and we can’t really
     write files directly there (too slow).
   - So we make a ring buffer (SPSC = single producer / single consumer).
   - Producer = camera callback drops frames into ring.
   - Consumer = writer thread pulls them out and saves to disk.
   - This way we don’t stall the camera pipeline.
---------------------------------------------------------------- */

#define RING 32   // number of slots in the ring (basically queue depth)

/* Each slot is one frame + some info */
typedef struct {
    uint8_t *ptr;   // pointer to actual frame data
    size_t   len;   // length of that frame in bytes
    int      idx;   // frame index (just counts frames)
    int64_t  pts;   // presentation timestamp (from MMAL)
} RING_SLOT_T;

/* global ring state */
static RING_SLOT_T g_ring[RING]; 
static volatile unsigned g_head = 0, g_tail = 0;   // head=write, tail=read
static unsigned g_max_occupancy = 0;               // just for stats/debug
static sem_t g_have_data;                          // signals writer when data arrives
static volatile int g_done = 0;                    // flag: stop writer
static size_t g_frame_bytes = 0;                   // how big each frame is
static int g_raw_fd = -1;                          // raw output file descriptor
static FILE *g_ts = NULL;                          // optional timestamp file
static pthread_t g_writer;                         // the background writer thread
static volatile int g_dropped = 0;                 // counter if frames got dropped
static int g_live_timestamps = 0;                  // live timestamp output flag

/* ---------------- the writer thread itself ----------------
   runs in background, waits for data in the ring, writes to file.
   - if ring is empty -> block on semaphore
   - when new frame comes -> grab slot, dump data to disk, 
     maybe log timestamps
   - loop until g_done is set AND ring is empty
---------------------------------------------------------------- */
static void *writer_thread(void *arg) 
{
    (void)arg;
    int64_t prev_pts = MMAL_TIME_UNKNOWN;  // keep track of previous timestamp

    while (!g_done || g_tail != g_head) {
        if (g_tail == g_head) {       // nothing in ring, so wait
            sem_wait(&g_have_data);
            continue;
        }
        RING_SLOT_T *s = &g_ring[g_tail];   // grab current slot

        // write raw frame to file (best effort, no checks here)
        if (g_raw_fd >= 0 && s->len) {
            ssize_t wrote = write(g_raw_fd, s->ptr, s->len);
            (void)wrote; // ignore errors, just dump and hope
        }

        // write timestamps if enabled
        if (g_ts) {
            if (prev_pts == MMAL_TIME_UNKNOWN) {
                // first line matches old post-process format: ",idx,pts"
                fprintf(g_ts, ",%d,%lld\n", s->idx, (long long)s->pts);
            } else {
                // later lines: delta,idx,pts
                fprintf(g_ts, "%lld,%d,%lld\n",
                        (long long)(s->pts - prev_pts), s->idx, (long long)s->pts);
            }
            prev_pts = s->pts;
        }

        // advance to next slot (circular buffer wrap-around with %RING)
        g_tail = (g_tail + 1) % RING;
    }

    // make sure timestamp file is flushed at the end
    if (g_ts) fflush(g_ts);
    return NULL;
}
/* ---------------- end of writer thread ---------------- */


void update_regs(const struct sensor_def *sensor, struct mode_def *mode, int hflip, int vflip, int exposure, int gain);

static int i2c_rd(int fd, uint8_t i2c_addr, uint16_t reg, uint8_t *values, uint32_t n, const struct sensor_def *sensor)
{
	int err;
	uint8_t buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_rdwr_ioctl_data msgset;
	struct i2c_msg msgs[2] = {
		{
		    .addr = i2c_addr,
		    .flags = 0,
		    .len = 2,
		    .buf = buf,
		},
		{
		    .addr = i2c_addr,
		    .flags = I2C_M_RD,
		    .len = n,
		    .buf = values,
		},
	};

	if (sensor->i2c_addressing == 1)
	{
		msgs[0].len = 1;
	}
	msgset.msgs = msgs;
	msgset.nmsgs = 2;

	err = ioctl(fd, I2C_RDWR, &msgset);
	// vcos_log_error("Read i2c addr %02X, reg %04X (len %d), value %02X,
	// err %d", i2c_addr, msgs[0].buf[0], msgs[0].len, values[0], err);
	if (err != (int)msgset.nmsgs)
		return -1;

	return 0;
}

const struct sensor_def *probe_sensor(int fd)
{
	const struct sensor_def **sensor_list = &sensors[0];
	const struct sensor_def *sensor = NULL;

	while (*sensor_list != NULL)
	{
		uint16_t reg = 0;
		sensor = *sensor_list;
		vcos_log_error("Probing sensor %s on addr %02X", sensor->name, sensor->i2c_addr);
		if (sensor->i2c_ident_length <= 2)
		{
			if (!i2c_rd(fd, sensor->i2c_addr, sensor->i2c_ident_reg, (uint8_t *)&reg,
				    sensor->i2c_ident_length, sensor))
			{
				if (reg == sensor->i2c_ident_value)
				{
					vcos_log_error("Found sensor %s at address %02X", sensor->name,
						       sensor->i2c_addr);
					break;
				}
			}
		}
		sensor_list++;
		sensor = NULL;
	}
	return sensor;
}

void send_regs(int fd, const struct sensor_def *sensor, const struct sensor_regs *regs, int num_regs)
{
	int i;
	for (i = 0; i < num_regs; i++)
	{
		if (regs[i].reg == 0xFFFF)
		{
			if (ioctl(fd, I2C_SLAVE_FORCE, regs[i].data) < 0)
			{
				vcos_log_error("Failed to set I2C address to %02X", regs[i].data);
			}
		}
		else if (regs[i].reg == 0xFFFE)
		{
			vcos_sleep(regs[i].data);
		}
		else
		{
			if (sensor->i2c_addressing == 1)
			{
				unsigned char msg[3] = { regs[i].reg, regs[i].data & 0xFF };
				int len = 2;

				if (sensor->i2c_data_size == 2)
				{
					msg[1] = (regs[i].data >> 8) & 0xFF;
					msg[2] = regs[i].data & 0xFF;
					len = 3;
				}
				if (write(fd, msg, len) != len)
				{
					vcos_log_error("Failed to write register index %d "
						       "(%02X val %02X)",
						       i, regs[i].reg, regs[i].data);
				}
			}
			else
			{
				unsigned char msg[4] = { regs[i].reg >> 8, regs[i].reg, regs[i].data };
				int len = 3;

				if (sensor->i2c_data_size == 2)
				{
					msg[2] = regs[i].data >> 8;
					msg[3] = regs[i].data;
					len = 4;
				}
				if (write(fd, msg, len) != len)
				{
					vcos_log_error("Failed to write register index %d", i);
				}
			}
		}
	}
}

void start_camera_streaming(const struct sensor_def *sensor, struct mode_def *mode, int fd)
{
	if (ioctl(fd, I2C_SLAVE_FORCE, sensor->i2c_addr) < 0)
	{
		vcos_log_error("Failed to set I2C address");
		return;
	}
	if (sensor->common_init)
		send_regs(fd, sensor, sensor->common_init, sensor->num_common_init);
	send_regs(fd, sensor, mode->regs, mode->num_regs);
	vcos_log_error("Now streaming...");
}

void stop_camera_streaming(const struct sensor_def *sensor, int fd)
{
	if (ioctl(fd, I2C_SLAVE_FORCE, sensor->i2c_addr) < 0)
	{
		vcos_log_error("Failed to set I2C address");
		return;
	}
	send_regs(fd, sensor, sensor->stop, sensor->num_stop_regs);
}

/**
 * Allocates and generates a filename based on the
 * user-supplied pattern and the frame number.
 * On successful return, finalName and tempName point to malloc()ed strings
 * which must be freed externally.  (On failure, returns nulls that
 * don't need free()ing.)
 *
 * @param finalName pointer receives an
 * @param pattern sprintf pattern with %d to be replaced by frame
 * @param frame for timelapse, the frame number
 * @return Returns a MMAL_STATUS_T giving result of operation
 */

MMAL_STATUS_T create_filenames(char **finalName, char *pattern, int frame)
{
	*finalName = NULL;
	if (0 > asprintf(finalName, pattern, frame))
	{
		return MMAL_ENOMEM; // It may be some other error, but it is not
				    // worth getting it right
	}
	return MMAL_SUCCESS;
}
static void buffers_to_rawcam(RASPIRAW_CALLBACK_T *dev)
{
	MMAL_BUFFER_HEADER_T *buffer;

	while ((buffer = mmal_queue_get(dev->rawcam_pool->queue)) != NULL)
	{
		mmal_port_send_buffer(dev->rawcam_output, buffer);
		// vcos_log_error("Buffer %p to rawcam\n", buffer);
	}
}


// ---------------- RAW CALLBACK (producer side) ----------------
// this runs every time the camera hands us a buffer.
// goal: get in, enqueue frame into our SPSC ring, get out fast,
// so we don't block the camera pipeline. 
static void callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    static int count = 0;  // counts every buffer that looks like a frame
    RASPIRAW_CALLBACK_T *dev = (RASPIRAW_CALLBACK_T *)port->userdata;
    RASPIRAW_PARAMS_T *cfg = (RASPIRAW_PARAMS_T *)dev->cfg;

    if (cfg->capture) // only do work if user actually asked to capture
    {
        // skip metadata-only buffers, and downsample using saverate
        if (!(buffer->flags & MMAL_BUFFER_HEADER_FLAG_CODECSIDEINFO) &&
            (((count++) % cfg->saverate) == 0))
        {
            /* shove frame into SPSC ring and bail quickly */
            unsigned h = g_head;              // where producer will write
            unsigned nxt = (h + 1) % RING;    // next head (wraps around)

            if (nxt != g_tail) {              // ring NOT full?
                RING_SLOT_T *s = &g_ring[h];
                size_t bytes = buffer->length;

                // safety: don’t overrun the pre-allocated slot buffer
                if (bytes > g_frame_bytes) bytes = g_frame_bytes;

                /* NOTE: buffer->user_data points to host memory we set up
                   in the pool. we copy from there into our ring slot.
                   (no file I/O here on purpose — too slow) */
                if (!cfg->write_empty) {
                    memcpy(s->ptr, buffer->user_data, bytes);
                    s->len = bytes;           // real payload length
                } else {
                    s->len = 0;               // “empty mode”: pretend write
                }

                s->idx = count - 1;           // remember which frame this was
                s->pts = buffer->pts;         // timestamp for later logging

                g_head = nxt;                 // publish the new head

                // track how “full” the ring got (just for nerdy stats)
                unsigned occ = (g_head >= g_tail)
                               ? (g_head - g_tail)
                               : (g_head + RING - g_tail);
                if (occ > g_max_occupancy) g_max_occupancy = occ;

                // poke the writer thread: “hey, there’s data now”
                sem_post(&g_have_data);
            } else {
                /* ring is full -> we don’t block (bad for camera),
                   so we just bump a drop counter. data is gone :/ */
                g_dropped++;
            }
        }
    }
    // we’re done with this MMAL buffer; return it so HW can reuse it ASAP
    mmal_buffer_header_release(buffer);

    // keep the rawcam port fed with empty buffers (prevents starvation)
    buffers_to_rawcam(dev);
}
// ---------------- end of raw callback ----------------


uint32_t order_and_bit_depth_to_encoding(enum bayer_order order, int bit_depth)
{
	// BAYER_ORDER_BGGR,
	// BAYER_ORDER_GBRG,
	// BAYER_ORDER_GRBG,
	// BAYER_ORDER_RGGB
	const uint32_t depth8[] = { MMAL_ENCODING_BAYER_SBGGR8, MMAL_ENCODING_BAYER_SGBRG8, MMAL_ENCODING_BAYER_SGRBG8,
				    MMAL_ENCODING_BAYER_SRGGB8 };
	const uint32_t depth10[] = { MMAL_ENCODING_BAYER_SBGGR10P, MMAL_ENCODING_BAYER_SGBRG10P,
				     MMAL_ENCODING_BAYER_SGRBG10P, MMAL_ENCODING_BAYER_SRGGB10P };
	const uint32_t depth12[] = {
		MMAL_ENCODING_BAYER_SBGGR12P,
		MMAL_ENCODING_BAYER_SGBRG12P,
		MMAL_ENCODING_BAYER_SGRBG12P,
		MMAL_ENCODING_BAYER_SRGGB12P,
	};
	const uint32_t depth16[] = {
		MMAL_ENCODING_BAYER_SBGGR16,
		MMAL_ENCODING_BAYER_SGBRG16,
		MMAL_ENCODING_BAYER_SGRBG16,
		MMAL_ENCODING_BAYER_SRGGB16,
	};
	if (order < 0 || order > 3)
	{
		vcos_log_error("order out of range - %d", order);
		return 0;
	}

	switch (bit_depth)
	{
	case 8:
		return depth8[order];
	case 10:
		return depth10[order];
	case 12:
		return depth12[order];
	case 16:
		return depth16[order];
	}
	vcos_log_error("%d not one of the handled bit depths", bit_depth);
	return 0;
}

/**
 * Parse the incoming command line and put resulting parameters in to the state
 *
 * @param argc Number of arguments in command line
 * @param argv Array of pointers to strings from command line
 * @param state Pointer to state structure to assign any discovered parameters
 * to
 * @return non-0 if failed for some reason, 0 otherwise
 */
static int parse_cmdline(int argc, char **argv, RASPIRAW_PARAMS_T *cfg)
{
	// Parse the command line arguments.
	// We are looking for --<something> or -<abbreviation of something>

	int valid = 1;
	int i;

	for (i = 1; i < argc && valid; i++)
	{
		int command_id, num_parameters, len;

		if (!argv[i])
			continue;

		if (argv[i][0] != '-')
		{
			valid = 0;
			continue;
		}

		// Assume parameter is valid until proven otherwise
		valid = 1;

		command_id =
		    raspicli_get_command_id(cmdline_commands, cmdline_commands_size, &argv[i][1], &num_parameters);

		// If we found a command but are missing a parameter, continue
		// (and we will drop out of the loop)
		if (command_id != -1 && num_parameters > 0 && (i + 1 >= argc))
			continue;

		//  We are now dealing with a command line option
		switch (command_id)
		{
		case CommandHelp:
			raspicli_display_help(cmdline_commands, cmdline_commands_size);
			// exit straight away if help requested
			return -1;

		case CommandMode:
			if (sscanf(argv[i + 1], "%d", &cfg->mode) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandHFlip:
			cfg->hflip = 1;
			break;

		case CommandVFlip:
			cfg->vflip = 1;
			break;

		case CommandExposure:
			if (sscanf(argv[i + 1], "%d", &cfg->exposure) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandGain:
			if (sscanf(argv[i + 1], "%d", &cfg->gain) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandOutput: // output filename
		{
			len = strlen(argv[i + 1]);
			if (len)
			{
				// We use sprintf to append the frame number for
				// timelapse mode Ensure that any %<char> is
				// either %% or %d.
				const char *percent = argv[i + 1];
				while (valid && *percent && (percent = strchr(percent, '%')) != NULL)
				{
					int digits = 0;
					percent++;
					while (isdigit(*percent))
					{
						percent++;
						digits++;
					}
					if (!((*percent == '%' && !digits) || *percent == 'd'))
					{
						valid = 0;
						fprintf(stderr, "Filename contains %% "
								"characters, but not "
								"%%d or %%%% - sorry, "
								"will fail\n");
					}
					percent++;
				}
				cfg->output = malloc(len + 10); // leave enough space for any timelapse
								// generated changes to filename
				if (cfg->output)
				{
					strncpy(cfg->output, argv[i + 1], len + 1);
					i++;
					cfg->capture = 1;
				}
				else
				{
					fprintf(stderr, "internal error - "
							"allocation fail\n");
					valid = 0;
				}
			}
			else
			{
				valid = 0;
			}
			break;
		}

		case CommandWriteHeader:
			cfg->write_header = 1;
			break;

		case CommandTimeout: // Time to run for in milliseconds
			if (sscanf(argv[i + 1], "%u", &cfg->timeout) == 1)
			{
				i++;
			}
			else
				valid = 0;
			break;

		case CommandSaveRate:
			if (sscanf(argv[i + 1], "%u", &cfg->saverate) == 1)
			{
				i++;
			}
			else
				valid = 0;
			break;

		case CommandBitDepth:
			if (sscanf(argv[i + 1], "%u", &cfg->bit_depth) == 1)
			{
				i++;
			}
			else
				valid = 0;
			break;


		case CommandExposureus:
			if (sscanf(argv[i + 1], "%d", &cfg->exposure_us) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandI2cBus:
			if (sscanf(argv[i + 1], "%d", &cfg->i2c_bus) != 1)
				valid = 0;
			else
				i++;
			break;


		case CommandRegs: // register changes
		{
			len = strlen(argv[i + 1]);
			cfg->regs = malloc(len + 1);
			vcos_assert(cfg->regs);
			strncpy(cfg->regs, argv[i + 1], len + 1);
			i++;
			break;
		}

		case CommandHinc:
			if (strlen(argv[i + 1]) != 2 || sscanf(argv[i + 1], "%x", &cfg->crop.hinc) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandVinc:
			if (strlen(argv[i + 1]) != 2 || sscanf(argv[i + 1], "%x", &cfg->crop.vinc) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandFps:
			if (sscanf(argv[i + 1], "%lf", &cfg->fps) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandWidth:
			if (sscanf(argv[i + 1], "%d", &cfg->crop.width) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandHeight:
			if (sscanf(argv[i + 1], "%d", &cfg->crop.height) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandLeft:
			if (sscanf(argv[i + 1], "%d", &cfg->crop.left) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandTop:
			if (sscanf(argv[i + 1], "%d", &cfg->crop.top) != 1)
				valid = 0;
			else
				i++;
			break;

		case CommandWriteHeader0:
			len = strlen(argv[i + 1]);
			cfg->write_header0 = malloc(len + 1);
			vcos_assert(cfg->write_header0);
			strncpy(cfg->write_header0, argv[i + 1], len + 1);
			i++;
			break;

		case CommandWriteHeaderG:
			len = strlen(argv[i + 1]);
			cfg->write_headerg = malloc(len + 1);
			vcos_assert(cfg->write_headerg);
			strncpy(cfg->write_headerg, argv[i + 1], len + 1);
			i++;
			break;

		case CommandWriteTimestamps:
			len = strlen(argv[i + 1]);
			cfg->write_timestamps = malloc(len + 1);
			vcos_assert(cfg->write_timestamps);
			strncpy(cfg->write_timestamps, argv[i + 1], len + 1);
			i++;
			cfg->ptsa = malloc(sizeof(*cfg->ptsa));
			cfg->ptso = cfg->ptsa;
			break;

		case CommandWriteEmpty:
			cfg->write_empty = 1;
			break;

		default:
			valid = 0;
			break;
		}
	}

	if (!valid)
	{
		fprintf(stderr, "Invalid command line option (%s)\n", argv[i - 1]);
		return 1;
	}

	return 0;
}

int main(int argc, char **argv)
{
	RASPIRAW_PARAMS_T cfg = { 0 };
	RASPIRAW_CALLBACK_T dev = { .cfg = &cfg, .rawcam_pool = NULL, .rawcam_output = NULL };
	uint32_t encoding;
	const struct sensor_def *sensor;
	struct mode_def *sensor_mode = NULL;
	char i2c_device_name[I2C_DEVICE_NAME_LEN];
	int i2c_fd;

	// Initialise any non-zero config values.
	cfg.exposure = -1;
	cfg.gain = -1;
	cfg.timeout = 5000;
	cfg.saverate = 20;
	cfg.bit_depth = -1;
	cfg.exposure_us = -1;
	cfg.i2c_bus = -1;
	cfg.crop.hinc = -1;
	cfg.crop.vinc = -1;
	cfg.fps = -1;
	cfg.crop.width = -1;
	cfg.crop.height = -1;
	cfg.crop.left = -1;
	cfg.crop.top = -1;

	bcm_host_init();
	vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);

	if (argc == 1)
	{
		fprintf(stdout, "\n%s Camera App %s\n\n", basename(argv[0]), VERSION_STRING);

		raspicli_display_help(cmdline_commands, cmdline_commands_size);
		exit(-1);
	}

	// Parse the command line and put options in to our status structure
	if (parse_cmdline(argc, argv, &cfg))
	{
		exit(-1);
	}

	if (cfg.i2c_bus == -1)
	{
		snprintf(i2c_device_name, sizeof(i2c_device_name), "/dev/i2c-%d", DEFAULT_I2C_DEVICE);
		i2c_fd = open(i2c_device_name, O_RDWR); 
	}
	else
	{
		snprintf(i2c_device_name, sizeof(i2c_device_name), "/dev/i2c-%d", cfg.i2c_bus);
		i2c_fd = open(i2c_device_name, O_RDWR);
	}

	if (!i2c_fd)
	{
		printf("Failed to open I2C device %s\n", i2c_device_name);
		return -1;
	}

	printf("Using I2C device %s\n", i2c_device_name);

	sensor = probe_sensor(i2c_fd);
	if (!sensor)
	{
		vcos_log_error("No sensor found. Aborting");
		return -1;
	}

	if (cfg.mode >= 0 && cfg.mode < sensor->num_modes)
	{
		sensor_mode = &sensor->modes[cfg.mode];
	}

	if (!sensor_mode)
	{
		vcos_log_error("Invalid mode %d - aborting", cfg.mode);
		return -2;
	}

	if (cfg.regs)
	{
		int r, b;
		char *p, *q;

		p = strtok(cfg.regs, ";");
		while (p)
		{
			vcos_assert(strlen(p) > 6);
			vcos_assert(p[4] == ',');
			vcos_assert(strlen(p) % 2);
			p[4] = '\0';
			q = p + 5;
			sscanf(p, "%4x", &r);
			while (*q)
			{
				vcos_assert(isxdigit(q[0]));
				vcos_assert(isxdigit(q[1]));

				sscanf(q, "%2x", &b);
				vcos_log_error("%04x: %02x", r, b);

				modReg(sensor_mode, r, 0, 7, b, EQUAL);

				++r;
				q += 2;
			}
			p = strtok(NULL, ";");
		}
	}

	if (cfg.crop.hinc >= 0 || cfg.crop.vinc >= 0 || cfg.crop.width > 0 || cfg.crop.width > 0 || cfg.crop.top >= 0 ||
	    cfg.crop.left >= 0)
	{
		if (sensor->set_crop)
		{
			if (sensor->set_crop(sensor, sensor_mode, &cfg.crop))
			{
				vcos_log_error("Failed setting manual crops. Aborting");
				return -1;
			}
		}
		else
		{
			vcos_log_error("This sensor does not currently support "
				       "manual cropping settings. Aborting");
			return -1;
		}
	}

	if (cfg.fps > 0)
	{
		int n = 1000000000 / (sensor_mode->line_time_ns * cfg.fps);
		modReg(sensor_mode, sensor->vts_reg + 0, 0, 7, n >> 8, EQUAL);
		modReg(sensor_mode, sensor->vts_reg + 1, 0, 7, n & 0xFF, EQUAL);
	}

	if (cfg.bit_depth == -1)
	{
		cfg.bit_depth = sensor_mode->native_bit_depth;
	}

	if (cfg.write_headerg && (cfg.bit_depth != sensor_mode->native_bit_depth))
	{
		// needs change after fix for
		// https://github.com/6by9/raspiraw/issues/2
		vcos_log_error("--headerG supported for native bit depth only");
		exit(-1);
	}

	if (cfg.exposure_us != -1)
	{
		cfg.exposure = ((int64_t)cfg.exposure_us * 1000) / sensor_mode->line_time_ns;
		vcos_log_error("Setting exposure to %d from time %dus", cfg.exposure, cfg.exposure_us);
	}

	update_regs(sensor, sensor_mode, cfg.hflip, cfg.vflip, cfg.exposure, cfg.gain);
	if (sensor_mode->encoding == 0)
		encoding = order_and_bit_depth_to_encoding(sensor_mode->order, cfg.bit_depth);
	else
		encoding = sensor_mode->encoding;
	if (!encoding)
	{
		vcos_log_error("Failed to map bitdepth %d and order %d into encoding\n", cfg.bit_depth,
			       sensor_mode->order);
		return -3;
	}
	vcos_log_error("Encoding %08X", encoding);


	MMAL_COMPONENT_T *rawcam = NULL, *isp = NULL, *render = NULL;
	MMAL_STATUS_T status;
	MMAL_PORT_T *output = NULL;
	MMAL_POOL_T *pool = NULL;
	MMAL_PARAMETER_CAMERA_RX_CONFIG_T rx_cfg;
	MMAL_PARAMETER_CAMERA_RX_TIMING_T rx_timing;
	unsigned int i;
	
	bcm_host_init();
	vcos_log_register("RaspiRaw", VCOS_LOG_CATEGORY);


	status = mmal_component_create("vc.ril.rawcam", &rawcam);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to create rawcam");
		return -1;
	}
	status = mmal_port_parameter_set_boolean(rawcam->output[0], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set zero copy");
		goto component_disable;
	}



	output = rawcam->output[0];

	rx_cfg.hdr.id = MMAL_PARAMETER_CAMERA_RX_CONFIG;
	rx_cfg.hdr.size = sizeof(rx_cfg);
	status = mmal_port_parameter_get(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to get cfg");
		goto component_destroy;
	}
	if (sensor_mode->encoding || cfg.bit_depth == sensor_mode->native_bit_depth)
	{
		rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
		rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_NONE;
	}
	else
	{
		switch (sensor_mode->native_bit_depth)
		{
		case 8:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_8;
			break;
		case 10:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_10;
			break;
		case 12:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_12;
			break;
		case 14:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_16;
			break;
		case 16:
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_16;
			break;
		default:
			vcos_log_error("Unknown native bit depth %d", sensor_mode->native_bit_depth);
			rx_cfg.unpack = MMAL_CAMERA_RX_CONFIG_UNPACK_NONE;
			break;
		}
		switch (cfg.bit_depth)
		{
		case 8:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_8;
			break;
		case 10:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_RAW10;
			break;
		case 12:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_RAW12;
			break;
		case 14:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_14;
			break;
		case 16:
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_16;
			break;
		default:
			vcos_log_error("Unknown output bit depth %d", cfg.bit_depth);
			rx_cfg.pack = MMAL_CAMERA_RX_CONFIG_PACK_NONE;
			break;
		}
	}
	vcos_log_error("Set pack to %d, unpack to %d", rx_cfg.unpack, rx_cfg.pack);
	if (sensor_mode->data_lanes)
		rx_cfg.data_lanes = sensor_mode->data_lanes;
	if (sensor_mode->image_id)
		rx_cfg.image_id = sensor_mode->image_id;
	status = mmal_port_parameter_set(output, &rx_cfg.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set cfg");
		goto component_destroy;
	}

	rx_timing.hdr.id = MMAL_PARAMETER_CAMERA_RX_TIMING;
	rx_timing.hdr.size = sizeof(rx_timing);
	status = mmal_port_parameter_get(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to get timing");
		goto component_destroy;
	}
	if (sensor_mode->timing[0])
		rx_timing.timing1 = sensor_mode->timing[0];
	if (sensor_mode->timing[1])
		rx_timing.timing2 = sensor_mode->timing[1];
	if (sensor_mode->timing[2])
		rx_timing.timing3 = sensor_mode->timing[2];
	if (sensor_mode->timing[3])
		rx_timing.timing4 = sensor_mode->timing[3];
	if (sensor_mode->timing[4])
		rx_timing.timing5 = sensor_mode->timing[4];
	if (sensor_mode->term[0])
		rx_timing.term1 = sensor_mode->term[0];
	if (sensor_mode->term[1])
		rx_timing.term2 = sensor_mode->term[1];
	vcos_log_error("Timing %u/%u, %u/%u/%u, %u/%u", rx_timing.timing1, rx_timing.timing2, rx_timing.timing3,
		       rx_timing.timing4, rx_timing.timing5, rx_timing.term1, rx_timing.term2);
	status = mmal_port_parameter_set(output, &rx_timing.hdr);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to set timing");
		goto component_destroy;
	}


	status = mmal_component_enable(rawcam);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable rawcam");
		goto component_destroy;
	}


	output->format->es->video.crop.width = sensor_mode->width;
	output->format->es->video.crop.height = sensor_mode->height;
	output->format->es->video.width = VCOS_ALIGN_UP(sensor_mode->width, 16);
	output->format->es->video.height = VCOS_ALIGN_UP(sensor_mode->height, 16);
	output->format->encoding = encoding;

	status = mmal_port_format_commit(output);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed port_format_commit");
		goto component_disable;
	}

	output->buffer_size = output->buffer_size_recommended;
	{ // Ensure we have at least a sensible number of buffers for rawcam output - it
	  // can go quite quickly. 
    unsigned wanted = 32; // 16 buffers at 1000fps is 62.5ms, at 500fps is 125ms
		// latency.  If the system is loaded, we may not get that, but try.
		// We use the recommended size as a minimum, in case the user is
		// using a very large frame size.	  
    output->buffer_num = (output->buffer_num_recommended < wanted)
                           ? wanted
                           : output->buffer_num_recommended;
	}

	if (cfg.capture)
	{
		if (cfg.write_header || cfg.write_header0)
		{
			brcm_header = (struct brcm_raw_header *)malloc(BRCM_RAW_HEADER_LENGTH);
			if (brcm_header)
			{
				memset(brcm_header, 0, BRCM_RAW_HEADER_LENGTH);
				brcm_header->id = BRCM_ID_SIG;
				brcm_header->version = HEADER_VERSION;
				brcm_header->mode.width = sensor_mode->width;
				brcm_header->mode.height = sensor_mode->height;
				// FIXME: Ought to check that the sensor is
				// producing Bayer rather than just assuming.
				brcm_header->mode.format = VC_IMAGE_BAYER;
				switch (sensor_mode->order)
				{
				case BAYER_ORDER_BGGR:
					brcm_header->mode.bayer_order = VC_IMAGE_BAYER_BGGR;
					break;
				case BAYER_ORDER_GBRG:
					brcm_header->mode.bayer_order = VC_IMAGE_BAYER_GBRG;
					break;
				case BAYER_ORDER_GRBG:
					brcm_header->mode.bayer_order = VC_IMAGE_BAYER_GRBG;
					break;
				case BAYER_ORDER_RGGB:
					brcm_header->mode.bayer_order = VC_IMAGE_BAYER_RGGB;
					break;
				}
				switch (cfg.bit_depth)
				{
				case 8:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW8;
					break;
				case 10:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW10;
					break;
				case 12:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW12;
					break;
				case 14:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW14;
					break;
				case 16:
					brcm_header->mode.bayer_format = VC_IMAGE_BAYER_RAW16;
					break;
				}
				if (cfg.write_header0)
				{
					// Save bcrm_header into one file only
					FILE *file;
					file = fopen(cfg.write_header0, "wb");
					if (file)
					{
						fwrite(brcm_header, BRCM_RAW_HEADER_LENGTH, 1, file);
						fclose(file);
					}
				}
			}
		}
		else if (cfg.write_headerg)
		{
			// Save pgm_header into one file only
			FILE *file;
			file = fopen(cfg.write_headerg, "wb");
			if (file)
			{
				fprintf(file, "P5\n%d %d\n255\n", sensor_mode->width, sensor_mode->height);
				fclose(file);
			}
		}
	}




	/* Set up the rawcam output callback */

	// Create buffer headers for rawcam to isp/awb/raw processing
	// Rawcam output.
	// Need to manually create the pool so that we have control over mem
	// handles, not letting MMAL core handle the magic.
	vcos_log_error("Create pool of %d buffers of size %d", output->buffer_num, output->buffer_size);
	pool = mmal_port_pool_create(output, output->buffer_num, 0);
	if (!pool)
	{
		vcos_log_error("Failed to create pool");
		goto component_disable;
	}
	for (i = 0; i < output->buffer_num; i++)
	{
		MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);
		if (!buffer)
			vcos_log_error("Where did my buffer go?");
		else
		{
			unsigned int vcsm_handle =
			    vcsm_malloc_cache(output->buffer_size, VCSM_CACHE_TYPE_HOST, "mmal_vc_port buffer");
			unsigned int vc_handle = vcsm_vc_hdl_from_hdl(vcsm_handle);
			uint8_t *mem = (uint8_t *)vcsm_lock(vcsm_handle);
			if (!mem || !vc_handle)
			{
				LOG_ERROR("could not allocate %i bytes of "
					  "shared memory (handle %x)",
					  (int)output->buffer_size, vcsm_handle);
				if (mem)
					vcsm_unlock_hdl(vcsm_handle);
				if (vcsm_handle)
					vcsm_free(vcsm_handle);
			}
			else
			{
				buffer->data = (void *)vc_handle;
				buffer->alloc_size = output->buffer_size;
				buffer->user_data = mem;
			}
		}
		mmal_buffer_header_release(buffer);
	}
	dev.rawcam_output = rawcam->output[0];
	dev.rawcam_pool = pool;

	/* --- Ring + writer thread init (after we know buffer_size) --- */
	// We allocate a ring of RING slots, each big enough to hold one
	// frame (worst case). The callback fills these in, the writer thread
	// pulls them off and writes them to disk. 
	g_frame_bytes = output->buffer_size;

	/* Pre-allocate ring slots */
	// Each slot gets a buffer of g_frame_bytes bytes.
	// We could be more clever and allocate only what we need for the 
	// current mode, but this is simpler and the memory use is not
	// too excessive for modern systems.
	for (i = 0; i < RING; ++i) {
		g_ring[i].ptr = (uint8_t*)malloc(g_frame_bytes);
		g_ring[i].len = 0; g_ring[i].idx = 0; g_ring[i].pts = 0;
	}
	sem_init(&g_have_data, 0, 0);
	g_head = g_tail = 0;
	g_done = 0;
	g_live_timestamps = (cfg.write_timestamps != NULL);

	/* Pre-open single RAW output file.
	If cfg->output contains '%%' format, fall back to a default path in /dev/shm. */
	// This is to avoid clobbering when doing timelapse. 
	// The writer thread will rename the file later. 
	if (cfg.capture) {
		const char *out_path = cfg.output;
		if (strchr(cfg.output, '%'))
			out_path = "/dev/shm/raspiraw_all.raw";
		g_raw_fd = open(out_path, O_CREAT|O_WRONLY|O_TRUNC, 0644);
	}

	/* Pre-open timestamps CSV (streaming append) */
	// If cfg->write_timestamps was given, we write timestamps as we go.
	// If not, we do nothing here.
	if (cfg.write_timestamps) {
		g_ts = fopen(cfg.write_timestamps, "wb");
	}

	/* Start writer thread */
	pthread_create(&g_writer, NULL, writer_thread, NULL);
	/* --- End of ring + writer thread init --- */



	output->userdata = (struct MMAL_PORT_USERDATA_T *)&dev;
	status = mmal_port_enable(output, callback);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to enable port");
		goto pool_destroy;
	}

	buffers_to_rawcam(&dev);

	start_camera_streaming(sensor, sensor_mode, i2c_fd);

	vcos_sleep(cfg.timeout);

	stop_camera_streaming(sensor, i2c_fd);

	/* --- Stop writer thread and clean up --- */
	// Signal thread to finish, wait for it to do so, then clean up.
	// We do this before we disable the port, so that the thread can
	// finish writing any data that might be in the ring buffer.
	// After this, the callback will never be called again.
	g_done = 1; 
	sem_post(&g_have_data);  // in case thread is waiting for data
	pthread_join(g_writer, NULL); 

	if (g_raw_fd >= 0) { close(g_raw_fd); g_raw_fd = -1; } 
	if (g_ts)          { fclose(g_ts);    g_ts = NULL; } 

	for (i = 0; i < RING; ++i) { 
		if (g_ring[i].ptr) { free(g_ring[i].ptr); g_ring[i].ptr = NULL; }
	} 
	sem_destroy(&g_have_data); 

	/* Free the single-node pts list allocated by -tstamps parsing (unused now) */
	if (cfg.ptsa) { free(cfg.ptsa); cfg.ptsa = NULL; }
	cfg.ptso = NULL;
	vcos_log_error("ring max occupancy: %u/%d, dropped: %d", g_max_occupancy, RING, g_dropped);

	// g_max_occupancy and g_dropped are only valid if we actually captured something.
	// If we did not capture anything, they will be zero, which is misleading.
	/* --- End of writer thread clean up --- */
port_disable:
	if (cfg.capture){
		status = mmal_port_disable(output);
		if (status != MMAL_SUCCESS)
		{
			vcos_log_error("Failed to disable port");
			return -1;
		}
	}
pool_destroy:
	if (pool)
		mmal_port_pool_destroy(output, pool);
component_disable:
	if (brcm_header)
		free(brcm_header);
	status = mmal_component_disable(rawcam);
	if (status != MMAL_SUCCESS)
	{
		vcos_log_error("Failed to disable rawcam");
	}
component_destroy:
	if (rawcam)
		mmal_component_destroy(rawcam);


	#define MAX_REASONABLE_PTS (1000000000000LL) // 1000s


	if (cfg.write_timestamps && !g_live_timestamps) {
	// Only save timestamps in offline mode.
    // In the old version this block ran whenever --tstamps was set;
    // now it is skipped if "live timestamps" are being written.
    FILE *file;
    file = fopen(cfg.write_timestamps, "wb");
    if (file) {
        int64_t old = 0;
        PTS_NODE_T aux;
        for (aux = cfg.ptsa; aux != cfg.ptso; aux = aux->nxt) {
            if (aux == cfg.ptsa) {
                fprintf(file, ",%d,%lld\n", aux->idx, aux->pts);
            } else {
				// Added plausibility checks.
                // Old code always wrote "<delta>,<idx>,<pts>".
                // Now we skip entries where:
                //   pts == MMAL_TIME_UNKNOWN
                //   pts decreased relative to 'old'
                //   pts <= 0
                //   pts exceeds MAX_REASONABLE_PTS (1e12 ns ~ 1000 s)
				// The last two are unlikely to happen, but if they do
				// they can mess up the entire CSV file.
				// The first two can happen if the camera is started
				// and stopped repeatedly, or if there is a glitch
				// in the camera link.
				// In these cases, we just skip the invalid/implausible
				// timestamp entry. 
                if (aux->pts != MMAL_TIME_UNKNOWN &&
                    old != MMAL_TIME_UNKNOWN &&
                    aux->pts >= old &&
                    aux->pts > 0 &&
                    aux->pts < MAX_REASONABLE_PTS) {
                    // Only output plausible and valid timestamps
                    fprintf(file, "%lld,%d,%lld\n", aux->pts - old, aux->idx, aux->pts);
                } else {
                    // Skip invalid or implausible timestamp entries
                    continue;
                }
            }
            old = aux->pts;
        }
        fclose(file);
    }
    while (cfg.ptsa != cfg.ptso) {
        PTS_NODE_T tmp = cfg.ptsa->nxt;
        free(cfg.ptsa);
        cfg.ptsa = tmp;
    }
    free(cfg.ptso);
	}

	close(i2c_fd);

	return 0;
}

void modRegBit(struct mode_def *mode, uint16_t reg, int bit, int value, enum operation op)
{
	int i = 0;
	uint16_t val;
	while (i < mode->num_regs && mode->regs[i].reg != reg)
		i++;
	if (i == mode->num_regs)
	{
		vcos_log_error("Reg: %04X not found!\n", reg);
		return;
	}
	val = mode->regs[i].data;

	switch (op)
	{
	case EQUAL:
		val = val & ~(1 << bit);
		val = val | (value << bit);
		break;
	case SET:
		val = val | (1 << bit);
		break;
	case CLEAR:
		val = val & ~(1 << bit);
		break;
	case XOR:
		val = val ^ (value << bit);
		break;
	}
	mode->regs[i].data = val;
}

void modReg(struct mode_def *mode, uint16_t reg, int startBit, int endBit, int value, enum operation op)
{
	int i;
	for (i = startBit; i <= endBit; i++)
	{
		modRegBit(mode, reg, i, value >> i & 1, op);
	}
}

void update_regs(const struct sensor_def *sensor, struct mode_def *mode, int hflip, int vflip, int exposure, int gain)
{
	if (sensor->vflip_reg)
	{
		modRegBit(mode, sensor->vflip_reg, sensor->vflip_reg_bit, vflip, XOR);
		if (vflip && !sensor->flips_dont_change_bayer_order)
			mode->order ^= 2;
	}

	if (sensor->hflip_reg)
	{
		modRegBit(mode, sensor->hflip_reg, sensor->hflip_reg_bit, hflip, XOR);
		if (hflip && !sensor->flips_dont_change_bayer_order)
			mode->order ^= 1;
	}

	if (sensor->exposure_reg && exposure != -1)
	{
		if (exposure < 0 || exposure >= (1 << sensor->exposure_reg_num_bits))
		{
			vcos_log_error("Invalid exposure:%d, exposure range is 0 to %u!\n", exposure,
				       (1 << sensor->exposure_reg_num_bits) - 1);
		}
		else
		{
			uint8_t val;
			int i, j = VCOS_ALIGN_DOWN(sensor->exposure_reg_num_bits - 1, 8);
			int num_regs = (sensor->exposure_reg_num_bits + 7) >> 3;
			for (i = 0; i < num_regs; i++, j -= 8)
			{
				val = (exposure >> j) & 0xFF;
				modReg(mode, sensor->exposure_reg + i, 0, 7, val, EQUAL);
				vcos_log_error("Set exposure %04X to %02X", sensor->exposure_reg + i, val);
			}
		}
	}
	if (sensor->vts_reg && exposure != -1 && exposure >= mode->min_vts)
	{
		if (exposure < 0 || exposure >= (1 << sensor->vts_reg_num_bits))
		{
			vcos_log_error("Invalid exposure:%d, vts range is 0 to %u!\n", exposure,
				       (1 << sensor->vts_reg_num_bits) - 1);
		}
		else
		{
			uint8_t val;
			int i, j = VCOS_ALIGN_DOWN(sensor->vts_reg_num_bits - 1, 8);
			int num_regs = (sensor->vts_reg_num_bits + 7) >> 3;

			for (i = 0; i < num_regs; i++, j -= 8)
			{
				val = (exposure >> j) & 0xFF;
				modReg(mode, sensor->vts_reg + i, 0, 7, val, EQUAL);
				vcos_log_error("Set vts %04X to %02X", sensor->vts_reg + i, val);
			}
		}
	}
	if (sensor->gain_reg && gain != -1)
	{
		if (gain < 0 || gain >= (1 << sensor->gain_reg_num_bits))
		{
			vcos_log_error("Invalid gain:%d, gain range is 0 to %u\n", gain,
				       (1 << sensor->gain_reg_num_bits) - 1);
		}
		else
		{
			uint8_t val;
			int i, j = VCOS_ALIGN_DOWN(sensor->gain_reg_num_bits - 1, 8);
			int num_regs = (sensor->gain_reg_num_bits + 7) >> 3;

			for (i = 0; i < num_regs; i++, j -= 8)
			{
				val = (gain >> j) & 0xFF;
				modReg(mode, sensor->gain_reg + i, 0, 7, val, EQUAL);
				vcos_log_error("Set gain %04X to %02X", sensor->gain_reg + i, val);
			}
		}
	}
}
