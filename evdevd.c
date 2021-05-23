/*
 * Copyright © 2013 Red Hat, Inc.
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that copyright
 * notice and this permission notice appear in supporting documentation, and
 * that the name of the copyright holders not be used in advertising or
 * publicity pertaining to distribution of the software without specific,
 * written prior permission.  The copyright holders make no representations
 * about the suitability of this software for any purpose.  It is provided "as
 * is" without express or implied warranty.
 *
 * THE COPYRIGHT HOLDERS DISCLAIM ALL WARRANTIES WITH REGARD TO THIS SOFTWARE,
 * INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS, IN NO
 * EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE,
 * DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THIS SOFTWARE.
 */

//#include "config.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "libevdev/libevdev.h"

#include <pthread.h>
#include <linux/uinput.h>

#include <getopt.h>

#include "fifo.h"

enum 
{
	CONTROLLER_OGA = 0, // RK2020 is same
	CONTROLLER_RG351P,
	NUM_OF_CONTROLLER
};

#define RG351V_RUMBLE

int g_iDetected_Controller=0;
int enableswapAB=0;
int g_iDPAD_rotate=0;
int g_ienableFF=0;
float g_fDeadzone=0;
float g_fSensitivity=1.0;
float g_fRumbleGain=1.0;
int enablelog = 0;
int g_iLeftAnalog_to_RightAnalog=0;
char g_strEmulator[256];
	
/* for FF */
#include <glib.h>
GIOChannel* m_io_channel;
int fd_uinput;
guint m_source_id;

gboolean on_read_data(GIOChannel* source, GIOCondition condition);

struct option longopts[] = {
	{ "help", no_argument, NULL, 'h' },
	{ "verbose", no_argument, NULL, 'v' },
	{ "grab", no_argument, NULL, 'g' },
	{ "emulator", required_argument, NULL, 'e' },
	{ "leftAStick2DPad", no_argument, NULL, 'l' },
	{ "swapAB", no_argument, NULL, 's' },
	{ "Deadzone", required_argument, NULL, 'D' },
	{ "Sensitivity", required_argument, NULL, 'S' },
	{ "ForceFeedback", no_argument, NULL, 'F' },
	{ "rumbleGain", required_argument, NULL, 'G' },
	{ "TestRumble", no_argument, NULL, 't' },
	{ 0, 0, 0, 0 }};

void display_help(const char *name)
{
	printf("%s [options]\n\n", name);
	printf("option lists\n");
	printf("-h,--help \t: display this message\n");
	printf("-v,--verbose \t: display debugging messages\n");
	printf("-g,--grab \t: grab the input device for uinput\n");
	printf("-e,--emulator (name of emulator) \t: do registered function for the emulator\n");
	printf("-l,--leftAStick2DPad \t: set left analog stick as dpad input\n");
	printf("-s,--swapAB \t: swap A and B\n");
	printf("-D,--Deadzone \t: set deadzone of the analog stick(default:0)\n");
	printf("-S,--Sensitivity \t: set sensitivity of th analog stick(default:1.0)\n");
	printf("-F,--ForceFeedback \t: enable force-feedback(rumble)\n");
	printf("-G,--rumbleGain \t: set gain for rumble(default:1.0)\n");
	printf("-t,--TestRumble \t: test rumble\n");
}

static void
print_abs_bits(struct libevdev *dev, int axis)
{
	const struct input_absinfo *abs;

	if (!libevdev_has_event_code(dev, EV_ABS, axis))
		return;

	abs = libevdev_get_abs_info(dev, axis);

	printf("	Value	%6d\n", abs->value);
	printf("	Min	%6d\n", abs->minimum);
	printf("	Max	%6d\n", abs->maximum);
	if (abs->fuzz)
		printf("	Fuzz	%6d\n", abs->fuzz);
	if (abs->flat)
		printf("	Flat	%6d\n", abs->flat);
	if (abs->resolution)
		printf("	Resolution	%6d\n", abs->resolution);
}

static void
print_code_bits(struct libevdev *dev, unsigned int type, unsigned int max)
{
	unsigned int i;
	for (i = 0; i <= max; i++) {
		if (!libevdev_has_event_code(dev, type, i))
			continue;

		printf("    Event code %i (%s)\n", i, libevdev_event_code_get_name(type, i));
		if (type == EV_ABS)
			print_abs_bits(dev, i);
	}
}

static void
print_bits(struct libevdev *dev)
{
	unsigned int i;
	printf("Supported events:\n");

	for (i = 0; i <= EV_MAX; i++) {
		if (libevdev_has_event_type(dev, i))
			printf("  Event type %d (%s)\n", i, libevdev_event_type_get_name(i));
		switch(i) {
			case EV_KEY:
				print_code_bits(dev, EV_KEY, KEY_MAX);
				break;
			case EV_REL:
				print_code_bits(dev, EV_REL, REL_MAX);
				break;
			case EV_ABS:
				print_code_bits(dev, EV_ABS, ABS_MAX);
				break;
			case EV_LED:
				print_code_bits(dev, EV_LED, LED_MAX);
				break;
		}
	}
}

static void
print_props(struct libevdev *dev)
{
	unsigned int i;
	printf("Properties:\n");

	for (i = 0; i <= INPUT_PROP_MAX; i++) {
		if (libevdev_has_property(dev, i))
			printf("  Property type %d (%s)\n", i,
					libevdev_property_get_name(i));
	}
}

static int
print_event(struct input_event *ev)
{
	if (ev->type == EV_SYN)
		printf("Event: time %ld.%06ld, ++++++++++++++++++++ %s +++++++++++++++\n",
				ev->input_event_sec,
				ev->input_event_usec,
				libevdev_event_type_get_name(ev->type));
	else
		printf("Event: time %ld.%06ld, type %d (%s), code %d (%s), value %d\n",
			ev->input_event_sec,
			ev->input_event_usec,
			ev->type,
			libevdev_event_type_get_name(ev->type),
			ev->code,
			libevdev_event_code_get_name(ev->type, ev->code),
			ev->value);
	return 0;
}

static int
print_sync_event(struct input_event *ev)
{
	printf("SYNC: ");
	print_event(ev);
	return 0;
}

unsigned int uiPowerButton=0;

void *threadPowerButtons(void *argumentPointer)
{
    struct libevdev *dev = (struct libevdev *)argumentPointer;
	int rc = 1;

	if (g_ienableFF)
	{
		static GMainLoop *m_gmain;

		m_gmain = g_main_loop_new(NULL, FALSE);

		// start g_io_channel
		m_io_channel = g_io_channel_unix_new(fd_uinput);

		// set encoding to binary
		GError* error = NULL;
		if (g_io_channel_set_encoding(m_io_channel, NULL, &error) != G_IO_STATUS_NORMAL)
		{
			printf("[trngaje] g_io_channel_set_encoding:error\n");
		}

		g_io_channel_set_buffered(m_io_channel, FALSE);

		m_source_id = g_io_add_watch(m_io_channel, G_IO_IN | G_IO_ERR | G_IO_HUP,
									 &on_read_data, NULL);	
									 
		g_main_loop_run(m_gmain);
	}

	
	do {
		struct input_event ev;
		
		rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL|LIBEVDEV_READ_FLAG_BLOCKING, &ev);
		if (rc == LIBEVDEV_READ_STATUS_SYNC) {

			while (rc == LIBEVDEV_READ_STATUS_SYNC) {
				//print_sync_event(&ev);
				rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_SYNC, &ev);
			}

		} else if (rc == LIBEVDEV_READ_STATUS_SUCCESS)
		{
			if (ev.type == EV_KEY)
			{
				if (ev.code == KEY_POWER)
				{
					if (ev.value != 0)
					{
						if (uiPowerButton == 0)
						{
							uiPowerButton = 1;
							printf("[trngaje] Power button is pressed\n");
						}
					}
					else
					{
						if (uiPowerButton == 1)
						{
							printf("[trngaje] Power button is released\n");
							uiPowerButton = 0;
						}
					}
				}
			}

			//print_event(&ev);
		}		
	} while (rc == LIBEVDEV_READ_STATUS_SYNC || rc == LIBEVDEV_READ_STATUS_SUCCESS || rc == -EAGAIN);

    return NULL;
}


void gotoxy(int x, int y)
{
    printf("%c[%d;%df", 0x1B, y, x);
}


typedef union  
{
	unsigned int val;
	struct 
	{
		unsigned int b0:1;
		unsigned int b1:1;
		unsigned int b2:1;
		unsigned int b3:1;
		unsigned int b4:1;
		unsigned int b5:1;
		unsigned int b6:1;
		unsigned int b7:1;		
		
		unsigned int b8:1;
		unsigned int b9:1;
		unsigned int b10:1;
		unsigned int b11:1;
		unsigned int b12:1;
		unsigned int b13:1;
		unsigned int b14:1;
		unsigned int b15:1;	
		
		unsigned int b16:1;
		unsigned int b17:1;
		unsigned int b18:1;
		unsigned int b19:1;
		unsigned int b20:1;
		unsigned int b21:1;
		unsigned int b22:1;
		unsigned int b23:1;	
		
		unsigned int b24:1;	
		unsigned int b25:1;	
		unsigned int b26:1;	
		unsigned int hup:1;
		unsigned int hdown:1;
		unsigned int hleft:1;
		unsigned int hright:1;	
		unsigned int power:1;	
	} bits;
	
	struct 
	{
		unsigned int a:1;
		unsigned int b:1;
		unsigned int x:1;
		unsigned int y:1;
		unsigned int l:1;
		unsigned int r:1;
		unsigned int start:1;
		unsigned int select:1;		
		
		unsigned int l3:1;
		unsigned int r3:1;
		unsigned int l2:1;
		unsigned int r2:1;
		unsigned int b12:1;
		unsigned int b13:1;
		unsigned int b14:1;
		unsigned int b15:1;	
		
		unsigned int b16:1;
		unsigned int b17:1;
		unsigned int b18:1;
		unsigned int b19:1;
		unsigned int b20:1;
		unsigned int b21:1;
		unsigned int b22:1;
		unsigned int b23:1;	
		
		unsigned int b24:1;	
		unsigned int b25:1;	
		unsigned int b26:1;	
		unsigned int hup:1;
		unsigned int hdown:1;
		unsigned int hleft:1;
		unsigned int hright:1;	
		unsigned int power:1;	
	} rg351p_joybit;	
	struct 
	{
		unsigned int joyb:27;
		unsigned int hud:2;
		unsigned int hlr:2;	
		unsigned int power:1;	
	} joyhatp;	
} joybutton32bit;

void emit(int fd, int type, int code, int val)
{
   struct input_event ie;

   ie.type = type;
   ie.code = code;
   ie.value = val;
   /* timestamp values below are ignored */
   ie.time.tv_sec = 0;
   ie.time.tv_usec = 0;

   write(fd, &ie, sizeof(ie));
}

void bindbuttons(int fd, int code, int value)
{
	// RG351P
	/*
    Event code 304 (BTN_SOUTH) a
    Event code 305 (BTN_EAST) b
    Event code 306 (BTN_C) x
    Event code 307 (BTN_NORTH) y
    Event code 308 (BTN_WEST) l
    Event code 309 (BTN_Z) r 
    Event code 310 (BTN_TL) start
    Event code 311 (BTN_TR) select
    Event code 312 (BTN_TL2) l3
    Event code 313 (BTN_TR2) r3
    Event code 314 (BTN_SELECT) l2
    Event code 315 (BTN_START) r2
    Event code 316 (BTN_MODE)
    Event code 317 (BTN_THUMBL)
    Event code 318 (BTN_THUMBR)
	*/
	int bindcode=0;
	
	if (g_iDetected_Controller == CONTROLLER_RG351P)
	{
		switch(code)
		{
			// RG351P
			case BTN_SOUTH: // a
				if (enableswapAB)
					bindcode = BTN_EAST;
				else
					bindcode = BTN_SOUTH;
				break;
			case BTN_EAST: // b
				if (enableswapAB)
					bindcode = BTN_SOUTH;
				else		
					bindcode = BTN_EAST;
				break;
			case BTN_C: // x
				bindcode = BTN_NORTH;
				break;
			case BTN_NORTH: // y
				bindcode = BTN_WEST;
				break;		
			case BTN_WEST: // l
				bindcode = BTN_TL;
				break;
			case BTN_Z: // r
				bindcode = BTN_TR;
				break;
			case BTN_TL: //start
				bindcode = BTN_TRIGGER_HAPPY4;//BTN_START;
				break;
			case BTN_TR: // select
				bindcode = BTN_TRIGGER_HAPPY3;//BTN_SELECT;
				break;
			case BTN_TL2: // l3
				bindcode = BTN_TRIGGER_HAPPY1; //BTN_THUMBL;
				break;
			case BTN_TR2: // r3
				bindcode = BTN_TRIGGER_HAPPY2; //BTN_THUMBR;
				break;	
			case BTN_SELECT: // l2
				bindcode = BTN_TL2;
				break;		
			case BTN_START: // r2
				bindcode = BTN_TR2;
				break;				
		};
	}
	else{
		switch(code)
		{
			// OGA
			case BTN_EAST: // a
				if (enableswapAB)
					bindcode = BTN_EAST;
				else
					bindcode = BTN_SOUTH;
				break;
			case BTN_SOUTH: // b
				if (enableswapAB)
					bindcode = BTN_SOUTH;
				else		
					bindcode = BTN_EAST;
				break;
			case BTN_NORTH: // x
				bindcode = BTN_NORTH;
				break;
			case BTN_WEST: // y
				bindcode = BTN_WEST;
				break;		
			case BTN_TL: // l
				bindcode = BTN_TL;
				break;
			case BTN_TR: // r
				bindcode = BTN_TR;
				break;
			case BTN_TRIGGER_HAPPY4: //start
				bindcode = BTN_START;
				break;
			case BTN_TRIGGER_HAPPY3: // select
				bindcode = BTN_SELECT;
				break;
			case BTN_TRIGGER_HAPPY1: // l3
				bindcode = BTN_THUMBL;
				break;
			case BTN_TRIGGER_HAPPY6: // r3
				bindcode = BTN_THUMBR;
				break;	
			case BTN_SELECT: // l2 : not used
				bindcode = BTN_TL2;
				break;		
			case BTN_START: // r2 : not used
				bindcode = BTN_TR2;
				break;				
		};		
	}
	
	
	if (bindcode != 0)
	{
		emit(fd, EV_KEY, bindcode, value);
		emit(fd, EV_SYN, SYN_REPORT, 0);	
	}
}

int enableLeftAStick2DPad = 0;
	
void transferabsvalues(int fd, int code, int value)
{
	int transcode=255;
	int transvalue;
	float fvalue;
	
	switch (code)
	{

	case ABS_Z: /* left analog */
		if (g_iLeftAnalog_to_RightAnalog == 1)
		{
			if (g_iDPAD_rotate == 1)
				transcode = ABS_RY;
			else
				transcode = ABS_RX;
		}
		else
			transcode = ABS_X;
		transvalue = 2048 - value;
		if (g_iDPAD_rotate == 1)
				transvalue = transvalue * (-1);
		break;
	case ABS_RX:
		if (g_iLeftAnalog_to_RightAnalog == 1)
		{
			if (g_iDPAD_rotate == 1)
				transcode = ABS_RX;
			else
				transcode = ABS_RY;
		}
		else
			transcode = ABS_Y;
		transvalue = 2048 - value;
	break;

	case ABS_RY:
		transcode = ABS_RX;
		transvalue = value - 2048;
	break;
	case ABS_RZ:
		transcode = ABS_RY;
		transvalue = value - 2048;
	break;
	}
	
	if (transcode != 255)
	{
		// for rg351p
		//fvalue = (float)transvalue / 512;
		//fvalue = fvalue * 2.5; 
		// for rg351v
		fvalue = (float)transvalue / 2048;
		fvalue = fvalue * g_fSensitivity;
		
		if (fvalue > 1.0) 
			fvalue = 1.0 ;
		if (fvalue < -1.0)
			fvalue = -1.0;
#if 1
		transvalue = 0x7FF * fvalue;
#else
		// for drastic
		transvalue = 0x80 + 0x7F * fvalue;
#endif

		
		if (enableLeftAStick2DPad && (code == ABS_Z || code == ABS_RX))
		{
			static int dpadlr=0;
			static int dpadud=0;
			if (fvalue > 0.3)
			{
				// right, down
				if (code == ABS_Z) // right
				{
					dpadlr = 1 ;
					emit(fd, EV_ABS, ABS_HAT0X, 1);
				}
				else // down
				{
					dpadud = 1 ;
					emit(fd, EV_ABS, ABS_HAT0Y, 1);
				}
			}
			else if (fvalue < -0.3)
			{
				// left, up
				if (code == ABS_Z) // left
				{
					dpadlr = -1 ;	
					emit(fd, EV_ABS, ABS_HAT0X, -1);
				}	
				else{ // up
					dpadud = -1 ;	
					emit(fd, EV_ABS, ABS_HAT0Y, -1);
				}
			}
			else
			{
				if (code == ABS_Z) // x
				{
					if (dpadlr != 0)
					{
						dpadlr = 0 ;	
						emit(fd, EV_ABS, ABS_HAT0X, 0);
					}
				}
				else // y
				{
					if (dpadud != 0)
					{
						dpadud = 0 ;	
						emit(fd, EV_ABS, ABS_HAT0Y, 0);
					}					
				}
			}
		}
		else
		{
			emit(fd, EV_ABS, transcode, transvalue);
		}
	}
	else
	{
		emit(fd, EV_ABS, code, value);
	}
	emit(fd, EV_SYN, SYN_REPORT, 0);	
}

struct libevdev *dev = NULL;

int openevdev(const char *file, struct libevdev *xdev)
{
	int fd;
	int rc = 1;
	
	fd = open(file, O_RDONLY);
	if (fd < 0) {
		perror("Failed to open device");
		
		return fd;
		//goto out;
	}

	rc = libevdev_new_from_fd(fd, &dev);
	if (rc < 0) {
		fprintf(stderr, "Failed to init libevdev (%s)\n", strerror(-rc));
		//goto out;
		
		return -1;
	}

	if (enablelog)
	{
		printf("Input device ID: bus %#x vendor %#x product %#x\n",
				libevdev_get_id_bustype(dev),
				libevdev_get_id_vendor(dev),
				libevdev_get_id_product(dev));
		printf("Evdev version: %x\n", libevdev_get_driver_version(dev));
		printf("Input device name: \"%s\"\n", libevdev_get_name(dev));
		printf("Phys location: %s\n", libevdev_get_phys(dev));
		printf("Uniq identifier: %s\n", libevdev_get_uniq(dev));
		print_bits(dev);
		print_props(dev);
	}
	
	return fd;
}


  
gboolean on_read_data(GIOChannel* source, GIOCondition condition)
{
	struct input_event ev;
	int ret;
	char cmds[256];
	int magnitude=0;
	
	while((ret = read(fd_uinput, &ev, sizeof(ev))) == sizeof(ev))
	{
		//sprintf(cmds, "echo %d,%d,%d >> /home/odroid/ff_all.txt", ev.type, ev.code, ev.value);
		//system(cmds);	
					
		switch(ev.type)
		{
		case EV_LED:
		if (ev.code == LED_MISC)
		{
		// FIXME: implement this
		//log_info("unimplemented: set LED status: " << ev.value);
		}
		break;

		case EV_FF:
			switch(ev.code)
			{
				case FF_RUMBLE:
					sprintf(cmds, "echo %d >> /home/odroid/ff_rumble.txt", ev.value);
					system(cmds);			
					break;

				case FF_GAIN:
					sprintf(cmds, "echo %d >> /home/odroid/ff_gain.txt", ev.value);
					system(cmds);			  
					//m_ff_handler->set_gain(ev.value);
					break;
				/*
				#define FF_STATUS_STOPPED	0x00
				#define FF_STATUS_PLAYING	0x01
				*/
/*
				case FF_STATUS_STOPPED:
					sprintf(cmds, "echo stopped:%d >> /home/odroid/ff_all.txt", ev.value);
					system(cmds);	
					break;
				case FF_STATUS_PLAYING:
					sprintf(cmds, "echo playing:%d >> /home/odroid/ff_all.txt", ev.value);
					system(cmds);	
					break;
*/					
				default:
					//sprintf(cmds, "echo %d,%d >> /home/odroid/ff_all.txt", ev.code, ev.value);
					//system(cmds);

					if (ev.value)
					{
						if (magnitude)
						{
						  sprintf(cmds, "echo %d > /sys/class/pwm/pwmchip0/pwm0/duty_cycle", 10000 - (int)(((float)magnitude / 0xffff) * 10000));
						  system(cmds);							
						}
						else{
						  sprintf(cmds, "echo %d > /sys/class/pwm/pwmchip0/pwm0/duty_cycle", 10/* - (int)(((float)0x5a / 255.0) * 1000000)*/);
						  system(cmds);
						}
					}
					else
					{
						  sprintf(cmds, "echo %d > /sys/class/pwm/pwmchip0/pwm0/duty_cycle", 10000 - (int)(((float)0 / 255.0) * 10000));
						  system(cmds);
					}						
					/*
					if (ev.value)
					m_ff_handler->play(ev.code);
					else
					m_ff_handler->stop(ev.code);

					*/
					break;
			}
			break;

		case EV_UINPUT:
			switch (ev.code)
			{
			case UI_FF_UPLOAD:
			{
			struct uinput_ff_upload upload;
			memset(&upload, 0, sizeof(upload));

			// *VERY* important, without this you break
			// the kernel and have to reboot due to dead
			// hanging process
			upload.request_id = ev.value;

			ioctl(fd_uinput, UI_BEGIN_FF_UPLOAD, &upload);
			//m_ff_handler->upload(upload.effect);
/*
struct uinput_ff_upload {
	int			request_id;
	int			retval;
	struct ff_effect	effect;
	struct ff_effect	old;
};			
*/	

#if 0
struct ff_effect {
        __u16 type;
        __s16 id;
        __u16 direction;
        struct ff_trigger trigger;
        struct ff_replay replay;

        union {
                struct ff_constant_effect constant;
                struct ff_ramp_effect ramp;
                struct ff_periodic_effect periodic;
                struct ff_condition_effect condition[2]; /* One for each axis */
                struct ff_rumble_effect rumble;
        } u;
};
#endif

/*
struct ff_rumble_effect {
        __u16 strong_magnitude;
        __u16 weak_magnitude;
};
*/
			// n64
			// upload:ff_periodic,0x5a,0x3e8,0x7fff,0x0,0x0

			if (upload.effect.type == FF_RUMBLE)
			{
				//sprintf(cmds, "echo upload:ff_rumble,%d, 0x%x,0x%x >> /home/odroid/ff_all.txt", upload.effect.id, upload.effect.u.rumble.strong_magnitude, upload.effect.u.rumble.weak_magnitude);
				//system(cmds);	
				if (upload.effect.id == 0)
				{
					magnitude= upload.effect.u.rumble.strong_magnitude * g_fRumbleGain;
					if (magnitude > 0xffff)
						magnitude = 0xffff;
				}
			}
			else if (upload.effect.type == FF_PERIODIC)
			{
				//sprintf(cmds, "echo upload:ff_periodic,%d, 0x%x,%d,0x%x,0x%x,0x%x >> /home/odroid/ff_all.txt", upload.retval, upload.effect.u.periodic.waveform,
				//	upload.effect.u.periodic.period, upload.effect.u.periodic.magnitude, upload.effect.u.periodic.offset, upload.effect.u.periodic.phase);
				//system(cmds);	
				magnitude=  upload.effect.u.periodic.magnitude * g_fRumbleGain;
				if (magnitude > 0xffff)
						magnitude = 0xffff;
			}
			else{
				sprintf(cmds, "echo upload:0x%x,0x%x,0x%x >> /home/odroid/ff_all.txt", upload.effect.type,upload.effect.u.rumble.strong_magnitude, upload.effect.u.rumble.weak_magnitude);
				system(cmds);					
			}
/*			
    case FF_PERIODIC:
      out << "FF_PERIODIC("
          << ", waveform:" << effect.u.periodic.waveform
          << ", period:" << effect.u.periodic.period
          << ", magnitude:" << effect.u.periodic.magnitude
          << ", offset:" << effect.u.periodic.offset
          << ", phase:" << effect.u.periodic.phase
          << ", envelope:" << effect.u.periodic.envelope << ")";
*/		  
			upload.retval = 0;

			ioctl(fd_uinput, UI_END_FF_UPLOAD, &upload);
			}
			break;

			case UI_FF_ERASE:
			{
			struct uinput_ff_erase erase;
			memset(&erase, 0, sizeof(erase));

			// *VERY* important, without this you break
			// the kernel and have to reboot due to dead
			// hanging process
			erase.request_id = ev.value;

			ioctl(fd_uinput, UI_BEGIN_FF_ERASE, &erase);
			//m_ff_handler->erase(erase.effect_id);
			erase.retval = 0;

			ioctl(fd_uinput, UI_END_FF_ERASE, &erase);
			}
			break;

			default:
			//log_warn("unhandled event code read");
			break;
			}
			break;

		default:
			//log_warn("unhandled event type read: " << ev.type);
			break;
		}
	}

	if (ret == 0)
	{
	// ok, no more data
			sprintf(cmds, "echo no more data >> /home/odroid/ff_all.txt");
			system(cmds);
	}
	else if (ret < 0)
	{
		if (errno != EAGAIN)
		{
		//log_error("failed to read from file description: " << ret << ": " << strerror(errno));
			sprintf(cmds, "echo failed to read:%d >> /home/odroid/ff_all.txt", ret);
			system(cmds);
		}
	}
	else
	{
	//log_error("short read: " << ret);
			sprintf(cmds, "echo short read:%d >> /home/odroid/ff_all.txt", ret);
			system(cmds);
	}

	return TRUE;
}

int
main(int argc, char **argv)
{
	//struct libevdev *dev = NULL;
	struct libevdev *dev_p = NULL;
	const char *file="/dev/input/event2";
	const char *emulator="";
	
	int fd, fd_p;
	int rc = 1, rc_p = 1;

    int c;
    int option_index = 0;
	int enablegrab = 0;


	
	while ((c = getopt_long(argc, argv, "e:D:S:vglFt", longopts, &option_index)) != -1)
	{ // optarg
		switch (c)
		{
			case 'v':	/* 디버깅 목적의 로그를 출력한다. */
				enablelog = 1;
				break;

			case 'g':	/* evdev를 grab 상태로 점유 한다. */ 
				enablegrab = 1;
				break;

			case 'e':
				emulator = optarg;
				break;
       
			case 'h':
				display_help(argv[0]);
				return;
				break;
			
			case 'l':
				enableLeftAStick2DPad = 1;
				break;
				
			case 's':
				enableswapAB = 1;
				break;
				
			case 'D':
				g_fDeadzone = atof(optarg);
				break;
			
			case 'S':
				g_fSensitivity = atof(optarg);
				break;

			case 'F':
				g_ienableFF = 1;
			break;
			
			case 'G':
				g_fRumbleGain = atof(optarg);
			break;

			case 't':
			
			break;
			
			default:
				printf("Unknown option. '%s'\n", longopts[option_index].name);
                //exit(EXIT_FAILURE);
				return;
		}
	}

	struct uinput_setup usetup;

	/*int */fd_uinput = open("/dev/uinput", O_RDWR/*O_WRONLY*/ | O_NONBLOCK);
    if (fd_uinput < 0) {
        printf("failed to open device %s\n", strerror(errno));
    }

	int version;
	rc = ioctl(fd_uinput, UI_GET_VERSION, &version);

	if (enablelog)
	{
		//[trngaje] version = 4
		printf("[trngaje] version = %d\n", version);
	}
	
#if 0
	if (rc == 0 && version >= 5) {
	  /* use UI_DEV_SETUP */
	  return 0;
	 
	}
#endif
	/*
	* The ioctls below will enable the device that is about to be
	* created, to pass key events, in this case the space key.
	*/
	ioctl(fd_uinput, UI_SET_EVBIT, EV_KEY);
	ioctl(fd_uinput, UI_SET_EVBIT, EV_ABS);
	//ioctl(fd_uinput, UI_SET_EVBIT, EV_MSC);
	// https://www.kernel.org/doc/html/v4.15/input/ff.html

	if (g_ienableFF)
	{
		ioctl(fd_uinput, UI_SET_EVBIT, EV_FF);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_RUMBLE);
		
		ioctl(fd_uinput, UI_SET_FFBIT, FF_PERIODIC);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_CONSTANT);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_RAMP);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_SINE);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_TRIANGLE);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_SQUARE);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_SAW_UP);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_SAW_DOWN);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_CUSTOM);
		ioctl(fd_uinput, UI_SET_FFBIT, FF_GAIN);
	}
	
	
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_A);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_D);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_M);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_F4);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_F5);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_F7);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_F8);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_PRINT);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_PAUSE);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_MENU);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_LEFTALT);
	ioctl(fd_uinput, UI_SET_KEYBIT, KEY_ESC);
	
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_EAST);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_SOUTH);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_NORTH);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_WEST);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_TL);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_TR);
/*
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_START);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_SELECT);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_THUMBL);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_THUMBR);
*/
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_TRIGGER_HAPPY1);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_TRIGGER_HAPPY2);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_TRIGGER_HAPPY3);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_TRIGGER_HAPPY4);
	
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_TL2);
	ioctl(fd_uinput, UI_SET_KEYBIT, BTN_TR2);
	
	ioctl(fd_uinput, UI_SET_ABSBIT, ABS_X);
	ioctl(fd_uinput, UI_SET_ABSBIT, ABS_Y);
	ioctl(fd_uinput, UI_SET_ABSBIT, ABS_RX);
	ioctl(fd_uinput, UI_SET_ABSBIT, ABS_RY);
	ioctl(fd_uinput, UI_SET_ABSBIT, ABS_HAT0X);
	ioctl(fd_uinput, UI_SET_ABSBIT, ABS_HAT0Y);	
	
	//ioctl(fd_uinput, UI_SET_MSCBIT, MSC_SCAN);	
#if 0
	// 아래와 같은 내용으로는 등록이 되지 않음.
	memset(&usetup, 0, sizeof(usetup));
	usetup.id.bustype = BUS_USB;
	usetup.id.vendor = 0x1; /* sample vendor */
	usetup.id.product = 0x10; /* sample product */
	strcpy(usetup.name, "trngaje_controller");
#endif
	struct uinput_user_dev uidev;
    memset(&uidev, 0, sizeof(uidev));
#if 1	
    // 정의되어 있는 이름으로 evtest event3에 표시되는 것 확인함
	snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "trngaje pad");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor  = 0x1;
    uidev.id.product = 0x10;
    uidev.id.version = 1;
#else
	snprintf(uidev.name, UINPUT_MAX_NAME_SIZE, "Microsoft X-Box 360 pad");
    uidev.id.bustype = BUS_USB;
    uidev.id.vendor  = 0x45e;
    uidev.id.product = 0x28e;
    uidev.id.version = 0x110;	
	
#endif
	uidev.absmin[ABS_X] = -2048;
	uidev.absmax[ABS_X] = 2047;
	uidev.absmin[ABS_Y] = -2048;
	uidev.absmax[ABS_Y] = 2047;

	uidev.absmin[ABS_RX] = -2048;
	uidev.absmax[ABS_RX] = 2047;
	uidev.absmin[ABS_RY] = -2048;
	uidev.absmax[ABS_RY] = 2047;

	uidev.absmin[ABS_HAT0X] = -1;
	uidev.absmax[ABS_HAT0X] = 1;
	uidev.absmin[ABS_HAT0Y] = -1;
	uidev.absmax[ABS_HAT0Y] = 1;
	
	if (g_ienableFF)
		uidev.ff_effects_max = 16; 
	
    if(write(fd_uinput, &uidev, sizeof(uidev)) < 0)
        printf("[trngaje] cannot init\n");
	
	//ioctl(fd_uinput, UI_DEV_SETUP, &usetup); // *

	ioctl(fd_uinput, UI_DEV_CREATE);
	
	// for RG351P
	system("echo 0 > /sys/class/pwm/pwmchip0/export");
	system("sudo chown -R odroid:odroid /sys/class/pwm/pwmchip0/pwm0");
	system("echo 10000 > /sys/class/pwm/pwmchip0/pwm0/period");
	system("echo 10000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle");
	system("echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable");

#if 0//def RG351V_RUMBLE
	static GMainLoop *m_gmain;

	m_gmain = g_main_loop_new(NULL, FALSE);

    // start g_io_channel
    m_io_channel = g_io_channel_unix_new(fd_uinput);

    // set encoding to binary
    GError* error = NULL;
    if (g_io_channel_set_encoding(m_io_channel, NULL, &error) != G_IO_STATUS_NORMAL)
    {
		printf("[trngaje] g_io_channel_set_encoding:error\n");
    }

    g_io_channel_set_buffered(m_io_channel, FALSE);

    m_source_id = g_io_add_watch(m_io_channel, G_IO_IN | G_IO_ERR | G_IO_HUP,
                                 &on_read_data, NULL);	
								 
	g_main_loop_run(m_gmain);


								 
	
/*							 
	m_source_id = g_io_create_watch (m_io_channel, G_IO_IN);
	g_source_set_callback (m_source_id,
						 (GSourceFunc) on_read_data,
						 NULL,
						 NULL);								 
*/	
#endif   
/*
	if (argc < 2)
		goto out;

	file = argv[1];
*/
	fd_p = open("/dev/input/event0", O_RDONLY);
	if (fd_p < 0) {
		perror("Failed to open device");
		goto out;
	}
	
	rc_p = libevdev_new_from_fd(fd_p, &dev_p);
	if (rc_p < 0) {
		fprintf(stderr, "Failed to init libevdev (%s)\n", strerror(-rc_p));
		goto out;
	}	

	if (enablelog)
	{
		printf("Input device ID: bus %#x vendor %#x product %#x\n",
				libevdev_get_id_bustype(dev_p),
				libevdev_get_id_vendor(dev_p),
				libevdev_get_id_product(dev_p));
		printf("Evdev version: %x\n", libevdev_get_driver_version(dev_p));
		printf("Input device name: \"%s\"\n", libevdev_get_name(dev_p));
		printf("Phys location: %s\n", libevdev_get_phys(dev_p));
		printf("Uniq identifier: %s\n", libevdev_get_uniq(dev_p));
		print_bits(dev_p);
		print_props(dev_p);
	}

/* */

	fd = openevdev(file, dev);
	//fd = open(file, O_RDONLY);
	if (fd < 0) {
		perror("Failed to open device");
		goto out;
	}
/*
	rc = libevdev_new_from_fd(fd, &dev);
	if (rc < 0) {
		fprintf(stderr, "Failed to init libevdev (%s)\n", strerror(-rc));
		goto out;
	}

	printf("Input device ID: bus %#x vendor %#x product %#x\n",
			libevdev_get_id_bustype(dev),
			libevdev_get_id_vendor(dev),
			libevdev_get_id_product(dev));
	printf("Evdev version: %x\n", libevdev_get_driver_version(dev));
	printf("Input device name: \"%s\"\n", libevdev_get_name(dev));
	printf("Phys location: %s\n", libevdev_get_phys(dev));
	printf("Uniq identifier: %s\n", libevdev_get_uniq(dev));
	print_bits(dev);
	print_props(dev);
*/
	if (strcmp(libevdev_get_name(dev), "OpenSimHardware OSH PB Controller") == 0)
	{
		g_iDetected_Controller = CONTROLLER_RG351P;
		if (enablelog)
			printf("[trngaje] RG351P_CONTROLLER is detected\n");
	}
	else
	{
		if (enablelog)
			printf("[trngaje] RG351P_CONTROLLER is %s\n", libevdev_get_name(dev));
	}
	
	if (enablegrab == 1)
	{
		if (enablelog)
			printf("[trngaje] enablegrab is true\n");
		libevdev_grab(dev, LIBEVDEV_GRAB);
	}

	system("sudo rm /dev/input/event2");
	system("sudo rm /dev/input/js0");

	if (enablelog)
		printf("[trngaje] emulator=%s\n", emulator);

	pthread_t threadPowerButtonsID;
    pthread_create(&threadPowerButtonsID, NULL, threadPowerButtons, (void*)dev_p);

	pthread_t threadGetFifoMessagesID;
    pthread_create(&threadGetFifoMessagesID, NULL, threadGetFifoMessages, NULL);

	int x=0, y=0, rightx=0, righty=0, hat0x=0, hat0y=0;
	unsigned int uiCurButtons=0, uiPrevButtons=0;
	joybutton32bit joyb, prevjoyb;
	joyb.val = 0;
	prevjoyb.val = 0;
	do {
		struct input_event ev, ev_p;
		rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL|LIBEVDEV_READ_FLAG_BLOCKING, &ev);
		if (rc == LIBEVDEV_READ_STATUS_SYNC) {

			//printf("::::::::::::::::::::: dropped ::::::::::::::::::::::\n");
			while (rc == LIBEVDEV_READ_STATUS_SYNC) {
				//print_sync_event(&ev);
				rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_SYNC, &ev);
			}
			//printf("::::::::::::::::::::: re-synced ::::::::::::::::::::::\n");

		} else if (rc == LIBEVDEV_READ_STATUS_SUCCESS)
		{
/*
			char *strenv_sdl2_ogs_vertical = __getenv("SDL2_OGS_VERTICAL");
			if (strenv_sdl2_ogs_vertical != NULL)
				g_iDPAD_rotate = atoi(strenv_sdl2_ogs_vertical);
*/
#if 0
			FILE *fp;
			char strenv_sdl2_ogs_vertical[10];

			/* Open the command for reading. */
			fp = popen("echo $SDL2_OGS_VERTICAL", "r");
			if (fp != NULL) {
				/* Read the output a line at a time - output it. */
				if (fgets(strenv_sdl2_ogs_vertical, sizeof(strenv_sdl2_ogs_vertical), fp) != NULL) {
					g_iDPAD_rotate = atoi(strenv_sdl2_ogs_vertical);
				}

				/* close */
				pclose(fp);		
			}
#endif
	
			
			if (ev.type == EV_ABS)
			{	
				
				if (ev.code == ABS_Z) /* left analog */
				{
					x = 2048 - ev.value;
					transferabsvalues(fd_uinput, ev.code, ev.value);
				}
				else if (ev.code == ABS_RX)
				{
					y = 2048 - ev.value;
					transferabsvalues(fd_uinput, ev.code, ev.value);
				}
				else if (ev.code == ABS_RY) /* right analog */
				{
					rightx = ev.value - 2048;
					transferabsvalues(fd_uinput, ev.code, ev.value);
				}
				else if (ev.code == ABS_RZ)
				{
					righty = ev.value - 2048;
					transferabsvalues(fd_uinput, ev.code, ev.value);
				}
				else if (ev.code == ABS_HAT0X)
				{
					hat0x = ev.value;
					
					if (ev.value < 0)
						joyb.joyhatp.hlr = 1;
					else if (ev.value > 0)
						joyb.joyhatp.hlr = 2;
					else
						joyb.joyhatp.hlr = 0;
					
					if (g_iDPAD_rotate == 1) 
						transferabsvalues(fd_uinput, ABS_HAT0Y, ev.value * (-1));
					else
						transferabsvalues(fd_uinput, ev.code, ev.value);
				}
				else if (ev.code == ABS_HAT0Y)
				{
					hat0y = ev.value;

					if (ev.value < 0)
						joyb.joyhatp.hud = 1;
					else if (ev.value > 0)
						joyb.joyhatp.hud = 2;
					else
						joyb.joyhatp.hud = 0;

					if (g_iDPAD_rotate == 1)
						transferabsvalues(fd_uinput, ABS_HAT0X, ev.value);
					else
						transferabsvalues(fd_uinput, ev.code, ev.value);					
				}
				
				//joyb.joyhatp.joyb = uiCurButtons;
				
				if (enablelog)
					printf("[trngaje] left_analog(%d,%d), right_analog(%d,%d), hat0(%d,%d)\n", x, y, rightx, righty, hat0x, hat0y);
				//gotoxy(40 + x / 100, 20 + y / 100); // left analog
				//gotoxy(40 + rightx / 100, 20 + righty / 100); // right analog
				//printf("#");
			}
			else if (ev.type == EV_KEY)
			{
				if (ev.code >= BTN_SOUTH && ev.code <= BTN_START)
				{
					if (ev.value == 0)
					{
						uiCurButtons &= ~(0x1 << (ev.code - BTN_SOUTH));
					}
					else
					{
						uiCurButtons |= 0x1 << (ev.code - BTN_SOUTH);
					}
					
					bindbuttons(fd_uinput, ev.code, ev.value);
/*					
					if (ev.code == BTN_START)
					{
						printf("[trngaje] r2 is pressed\n");
						emit(fd_uinput, EV_KEY, KEY_ESC, ev.value);
						emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
					}
*/					
					
					//joyb.val = uiCurButtons;
					joyb.joyhatp.joyb = uiCurButtons;
				}
				

			}

/*
			else if (ev.type == EV_MSC)
			{
				if (ev.code >= MSC_SCAN) 
					emit(fd_uinput, EV_MSC, MSC_SCAN, ev.value);
			}
*/
			//else
				//print_event(&ev				
			if (prevjoyb.val != joyb.val)
			{
				char *strenv;
				
				//printf("[trngaje] joy_button(%d):0x%x\n", ev.code, uiCurButtons);
				if (enablelog)
					printf("[trngaje] a=%d,b=%d,x=%d,y=%d,l=%d,r=%d,select=%d,start=%d,l2=%d,r2=%d,l3=%d,r3=%d, hat(%d,%d,%d,%d)\n",
					joyb.rg351p_joybit.a, joyb.rg351p_joybit.b, joyb.rg351p_joybit.x, joyb.rg351p_joybit.y, joyb.rg351p_joybit.l, joyb.rg351p_joybit.r, 
					joyb.rg351p_joybit.select, joyb.rg351p_joybit.start, joyb.rg351p_joybit.l2, joyb.rg351p_joybit.r2, joyb.rg351p_joybit.l3, joyb.rg351p_joybit.r3,
					joyb.rg351p_joybit.hleft, joyb.rg351p_joybit.hright, joyb.rg351p_joybit.hup, joyb.rg351p_joybit.hdown);
				//int ret;
				//ret = system("pgrep -f drastic");
				//printf("[trngaje] drastic exist =%d\n", ret);
				
				if (joyb.rg351p_joybit.r3 == 1)
				{
					//strenv = getenv("EMULATOR");
					//printf("[trngaje] EMULATOR=%s\n", strenv);

					if (joyb.rg351p_joybit.hleft == 1)
					{
						//printf("[trngaje] hotkey + DPADleft\n");
						system("/usr/bin/amixer -q sset Playback 5%-");
					}
					else if (joyb.rg351p_joybit.hright == 1)
					{
						//printf("[trngaje] hotkey + DPADright\n");
						system("/usr/bin/amixer -q sset Playback 5%+");
					}
					else if (joyb.rg351p_joybit.hup == 1)
					{
						if (enablelog)
							printf("[trngaje] hotkey + DPADup\n");
					}
					else if (joyb.rg351p_joybit.hdown == 1)
					{
						if (enablelog)
							printf("[trngaje] hotkey + DPADdown\n");
					}
					else if (joyb.rg351p_joybit.l3 == 1) // exit
					{
						if (enablelog)
							printf("[trngaje] hotkey + l3\n");
					}
					
					if (strcmp(emulator, "drastic.sh")==0)
					{
						printf("[trngaje] drastic\n");
						//drastic="--ui-buttonmap rt+lb=KEY_F7,rt+rb=KEY_F5,rt+x=KEY_M,rt+y=KEY_D,rt+b=KEY_A,rt+lt=KEY_ESC"
						if (joyb.rg351p_joybit.l)
						{
							emit(fd_uinput, EV_KEY, KEY_F7, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_F7, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}
						else if (joyb.rg351p_joybit.r)
						{
							emit(fd_uinput, EV_KEY, KEY_F5, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_F5, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}
						else if (joyb.rg351p_joybit.x)
						{
							emit(fd_uinput, EV_KEY, KEY_M, 1);
							//emit(fd_uinput, EV_KEY, KEY_M, 0);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_M, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}
						else if (joyb.rg351p_joybit.y)
						{
							emit(fd_uinput, EV_KEY, KEY_D, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_D, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}
						else if (joyb.rg351p_joybit.b)
						{
							emit(fd_uinput, EV_KEY, KEY_A, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_A, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}
						else if (joyb.rg351p_joybit.l3)
						{
							emit(fd_uinput, EV_KEY, KEY_ESC, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_ESC, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}
					}
					else if (strcmp(emulator, "openmsx")==0)
					{
						//openmsx="--ui-buttonmap rt+tl=KEY_PRINT,rt+y=KEY_PAUSE,rt+x=KEY_MENU,rt+lt=KEY_LEFTALT+KEY_F4,rt+lb=KEY_LEFTALT+KEY_F7,rt+rb=KEY_LEFTALT+KEY_F8"
						if (joyb.rg351p_joybit.l2)
						{
							emit(fd_uinput, EV_KEY, KEY_PRINT, 1);
							//emit(fd_uinput, EV_KEY, KEY_PRINT, 0);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_PRINT, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}		
						else if (joyb.rg351p_joybit.y)
						{
							emit(fd_uinput, EV_KEY, KEY_PAUSE, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_PAUSE, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}	
						else if (joyb.rg351p_joybit.x)
						{
							emit(fd_uinput, EV_KEY, KEY_MENU, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_MENU, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}	
						else if (joyb.rg351p_joybit.l3)
						{
							emit(fd_uinput, EV_KEY, KEY_LEFTALT, 1);
							emit(fd_uinput, EV_KEY, KEY_F4, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_LEFTALT, 0);
							//emit(fd_uinput, EV_KEY, KEY_F4, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}	
						else if (joyb.rg351p_joybit.l)
						{
							emit(fd_uinput, EV_KEY, KEY_LEFTALT, 1);
							emit(fd_uinput, EV_KEY, KEY_F7, 1);

							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_LEFTALT, 0);
							//emit(fd_uinput, EV_KEY, KEY_F7, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}	
						else if (joyb.rg351p_joybit.r)
						{
							emit(fd_uinput, EV_KEY, KEY_LEFTALT, 1);
							emit(fd_uinput, EV_KEY, KEY_F8, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							//emit(fd_uinput, EV_KEY, KEY_LEFTALT, 0);
							//emit(fd_uinput, EV_KEY, KEY_F8, 0);
							//emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}						
					}
					else if (strcmp(emulator, "PPSSPPSDL")==0)
					{
						if (joyb.rg351p_joybit.l3)
						{
							emit(fd_uinput, EV_KEY, KEY_ESC, 1);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);
							emit(fd_uinput, EV_KEY, KEY_ESC, 0);
							emit(fd_uinput, EV_SYN, SYN_REPORT, 0);								
						}						
					}



				
				}
				prevjoyb.val = joyb.val;
				
			}
			
		}

		if (rc != LIBEVDEV_READ_STATUS_SYNC && rc != LIBEVDEV_READ_STATUS_SUCCESS && rc != -EAGAIN)
		{
			// 에러가 있는 경우
			printf("[trngaje] rc = %d", rc);
			libevdev_free(dev);
			close(fd);			
			
			do{
				fd = open(file, O_RDONLY);
				if (fd < 0)
				{
					//printf("Joystick: No gamepad found.\n");
				}
				else
				{    
					if (libevdev_new_from_fd(fd, &dev) < 0)
					{
						printf("Joystick: Failed to init libevdev (%s)\n", strerror(-rc));
					}
				}	
			} while (fd < 0);
			rc = LIBEVDEV_READ_STATUS_SUCCESS; 

			if (enablegrab == 1)
			{
				if (enablelog)
					printf("[trngaje] enablegrab is true\n");
				libevdev_grab(dev, LIBEVDEV_GRAB);
			}
			system("sudo rm /dev/input/event2");
			system("sudo rm /dev/input/js0");
		}
	} while (rc == LIBEVDEV_READ_STATUS_SYNC || rc == LIBEVDEV_READ_STATUS_SUCCESS || rc == -EAGAIN);

	if (rc != LIBEVDEV_READ_STATUS_SUCCESS && rc != -EAGAIN)
		fprintf(stderr, "Failed to handle events: %s\n", strerror(-rc));

	rc = 0;
out:
	libevdev_free(dev);
	libevdev_free(dev_p);
	
	ioctl(fd_uinput, UI_DEV_DESTROY);
	close(fd_uinput);	
	return rc;
}
