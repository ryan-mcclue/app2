// SPDX-License-Identifier: zlib-acknowledgement

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xrandr.h>
#include <X11/extensions/Xrender.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <unistd.h>
#include <fcntl.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <dirent.h>

#include <linux/input.h>

#include "app-defs.h"
#include "app.c"

typedef struct {
  char *name;
  u64 time_ms;
} TimedBlock;

extern TimedBlock timed_blocks[];

void
end_timed_block(TimedBlock **timed_block)
{
  struct timespec ts = {0};
  if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts) == -1)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    EBP();
  }

  (*timed_block)->time_ms = (ts.tv_nsec / 1000.f) - 
                            (*timed_block)->time_ms;
}

#define CREATE_TIMED_BLOCK(n) \
  TimedBlock *timed_block __attribute__((cleanup(end_timed_block))) = \
      &timed_blocks[__COUNTER__]; \
  timed_block->name = n; \
  struct timespec ts = {0}; \
  if (clock_gettime(CLOCK_MONOTONIC_RAW, &ts) == -1) { EBP(); } \
  timed_block->time_ms = (ts.tv_nsec / 1000.f); \

struct linux_dirent64 {
  // IMPORTANT(Ryan): Prepending '__' to kernel typedef may make it
  // available
  __ino64_t d_ino;
  __off64_t d_off;
  unsigned short d_reclen;
  unsigned char  d_type;
  char d_name[];
};

#define EVDEV_BITFIELD_QUANTA \
  (sizeof(unsigned long) * 8)
#define EVDEV_BITFIELD_LEN(bit_count) \
  ((bit_count) / EVDEV_BITFIELD_QUANTA + 1)
#define EVDEV_BITFIELD_TEST(bitfield, bit) \
  (((bitfield)[(bit) / EVDEV_BITFIELD_QUANTA] >> \
     ((bit) % EVDEV_BITFIELD_QUANTA)) & 0x1)

INTERNAL int
xlib_error_handler(Display *display, XErrorEvent *err)
{
  char msg_type[32] = {0};
  snprintf(msg_type, sizeof(msg_type), "%d", err->error_code);
  char protocol_request_buf[512] = {0};
	XGetErrorDatabaseText(display, "XRequest", msg_type, "[NOT FOUND]", 
                        protocol_request_buf, 
                        sizeof(protocol_request_buf));

  // TODO(INVESTIGATE | Ryan): What dictates buffer size?
  char err_msg_buf[1024] = {0};
  XGetErrorText(display, err->error_code, err_msg_buf, 
                sizeof(err_msg_buf));

  BP();

  return 1;
}

INTERNAL int
xlib_io_error_handler(Display *display)
{
  BP();
}

// TODO(PROD | Ryan): Handle variable refresh rate monitors
INTERNAL uint
xrandr_get_active_refresh_rate(Display *display, Window root_window)
{
  CREATE_TIMED_BLOCK("refresh_rate");
  XRRScreenResources *screen_resources = \
      XRRGetScreenResources(display, root_window);
  if (screen_resources == NULL)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    BP();
  }

  RRMode active_mode_id = 0;
  for (uint crtc_num = 0; 
       crtc_num < screen_resources->ncrtc;
       ++crtc_num) 
  {
    XRRCrtcInfo *crtc_info = \
        XRRGetCrtcInfo(display, 
                       screen_resources, 
                       screen_resources->crtcs[crtc_num]);
    if (crtc_info == NULL)
    {
      // TODO(EHANDLING | ELOGGING: Ryan)
      BP();
    }
    if (crtc_info->mode != None)
    {
      active_mode_id = crtc_info->mode;
      break;
    }
  }

  uint refresh_rate = 0;
  for (uint mode_num = 0; 
       mode_num < screen_resources->nmode; 
       ++mode_num) 
  {
    XRRModeInfo mode_info = screen_resources->modes[mode_num];
    if (mode_info.id == active_mode_id)
    {
      // TODO(INVESTIGATE | Ryan): When to cast?
      refresh_rate = (r32)mode_info.dotClock / 
                     ((r32)mode_info.hTotal * (r32)mode_info.vTotal);
    }
  }

  return refresh_rate;
}

INTERNAL int
evdev_get_input_devices(int epoll_fd)
{
  CREATE_TIMED_BLOCK("evdev_get_devices");
  struct linux_dirent64 *dirent = NULL;
  int input_dir_fd = open("/dev/input", O_RDONLY | O_DIRECTORY);
  if (input_dir_fd == -1)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    EBP();
    return -1;
  }

  char input_dir_buf[4096] = {0};
  int bytes_read = 0;
  uint total_bytes_read = 0;
  do
  {
    bytes_read = syscall(SYS_getdents64, input_dir_fd, 
                             input_dir_buf, sizeof(input_dir_buf));
    if (bytes_read == -1)
    {
      // TODO(EHANDLING | ELOGGING: Ryan)
      EBP();
      return -1;
    }

    total_bytes_read += bytes_read;

  } while (bytes_read != 0);

  uint dirent_pos = 0;
  uint num_devices_added = 0;
  while (dirent_pos < total_bytes_read)
  {
    struct linux_dirent64 *dirent64 = \
        (struct linux_dirent64 *)(input_dir_buf + dirent_pos);
    if (strncmp(dirent64->d_name, "event", 5) == 0)
    {
      char dev_path[256] = {0};
      strcpy(dev_path, "/dev/input/");
      strcat(dev_path, dirent64->d_name);

      // NOTE(Ryan): Default user won't have access to /dev/input.
      // Remediate with $(usermod -a $(whoami) -G input)
      int dev_fd = open(dev_path, O_RDONLY);
      if (dev_fd == -1)
      {
        // TODO(EHANDLING | ELOGGING: Ryan)
        EBP();
        return -1;
      }

      unsigned long key_bitmask[EVDEV_BITFIELD_LEN(KEY_CNT)] = {0};

      // NOTE(Ryan): This is shorthand for reading kernel pseudo
      // file "/sys/class/input/inputX/capabilities/key". fstat will
      // give wrong file size, so have to read as many bytes as
      // available if taking this route.
      if (ioctl(dev_fd, EVIOCGBIT(EV_KEY, KEY_CNT), key_bitmask) == -1)
      {
        // TODO(EHANDLING | ELOGGING: Ryan)
        EBP();
        return -1;
      }

      if (key_bitmask[0] & 0xfffffffe)
      {
        char dev_name[256] = {0};
        if (ioctl(dev_fd, EVIOCGNAME(sizeof(dev_name)), 
                  dev_name) == -1)
        {
          // TODO(EHANDLING | ELOGGING: Ryan)
          EBP();
          return -1;
        }
        printf("Found keyboard: %s\n", dev_name);

        struct epoll_event event = {0};
        event.events = EPOLLIN;
        event.data.fd = dev_fd;
        if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, dev_fd, &event))
        {
          // TODO(EHANDLING | ELOGGING: Ryan)
          EBP();
          return -1;
        }
      }
      // TODO(PROD | Ryan): Stringent capability checking by comparing
      // $(evtest)
      else if (EVDEV_BITFIELD_TEST(key_bitmask, BTN_GAMEPAD))
      {
        char dev_name[256] = {0};
        if (ioctl(dev_fd, EVIOCGNAME(sizeof(dev_name)), 
                  dev_name) == -1)
        {
          // TODO(EHANDLING | ELOGGING: Ryan)
          EBP();
          return -1;
        }
        printf("Found gamepad: %s\n", dev_name);

        struct epoll_event event = {0};
        event.events = EPOLLIN;
        // TODO(INVESTIGATE | Ryan): Can this be passed user-defined
        // data?
        event.data.fd = dev_fd;
        if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, dev_fd, &event))
        {
          // TODO(EHANDLING | ELOGGING: Ryan)
          EBP();
          return -1;
        }
      } 
      else {
        close(dev_fd);
      }

    }
    dirent_pos += dirent64->d_reclen;
  }

  close(input_dir_fd);

  return 0;
}

INTERNAL void
xrender_display_pixel_buffer(Display *display,
                             XRenderPictFormat *pict_format,
                             Pixmap pixmap, XImage *image,
                             Window window, uint window_width, 
                             uint window_height)
{
  CREATE_TIMED_BLOCK("display_buffer");
  GC gc = DefaultGC(display, DefaultScreen(display));
  XPutImage(display, pixmap, gc, image, 
          0, 0, 0, 0, image->width, image->height);
  
  XRenderPictureAttributes pict_attributes = {0};
  Picture src_pict = XRenderCreatePicture(display, pixmap, 
                                          pict_format, 0, 
                                          &pict_attributes);
  Picture dst_pict = XRenderCreatePicture(display, window, 
                                          pict_format, 0, 
                                          &pict_attributes);
  
  double x_scale = image->width / window_width;
  double y_scale = image->height / window_height;
  XTransform transform_matrix = {{
    {XDoubleToFixed(x_scale), XDoubleToFixed(0), XDoubleToFixed(0)},
    {XDoubleToFixed(0), XDoubleToFixed(y_scale), XDoubleToFixed(0)},
    {XDoubleToFixed(0), XDoubleToFixed(0), XDoubleToFixed(1.0)}  
  }};
  XRenderSetPictureTransform(display, src_pict, &transform_matrix);
  
  XRenderComposite(display, PictOpSrc, src_pict, 0, dst_pict, 
                  0, 0, 0, 0, 0, 0,
                  window_width, window_height);
}

int
main(int argc, char *argv[])
{
  // NOTE(Ryan): Display string format is
  // <hostname:sequence_num:screen_num>
  // NOTE(Ryan): Comparing PIDs with $(htop) and $(ipcs -p), 
  // Xlib will create a shared memory segment to use with Xorg server
  Display *xlib_display = XOpenDisplay(NULL);
  if (xlib_display == NULL)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    BP();
  }

  XSetErrorHandler(xlib_error_handler);
  XSetIOErrorHandler(xlib_io_error_handler);

  Window xlib_root_window = XDefaultRootWindow(xlib_display);
  // NOTE(Ryan): A screen is a buffer that can be rendered to
  int xlib_screen = XDefaultScreen(xlib_display);

  // NOTE(Ryan): True Colour uses 24bits
  int screen_bit_depth = 24;
  XVisualInfo xlib_visual_info = {0};
  Status vis_info_status = XMatchVisualInfo(xlib_display, 
                                            xlib_screen, 
                                            screen_bit_depth,
                                            TrueColor, 
                                            &xlib_visual_info);
  if (vis_info_status == False)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    BP();
  }

  XSetWindowAttributes xlib_window_attr = {0};
  xlib_window_attr.background_pixel = BlackPixel(xlib_display, 
                                            xlib_screen);
  xlib_window_attr.event_mask = StructureNotifyMask;
  // TODO(PROD | Ryan): Control initial window position as controlled
  // by the window manager
  int top_left_x = 0; 
  int top_left_y = 0;
  uint window_width = 1280;
  uint window_height = 720;
  uint border_width = 0;
  Window xlib_window = XCreateWindow(xlib_display, xlib_root_window,
                                     top_left_x, top_left_y,
                                     window_width, window_height, 
                                     border_width,
                                     xlib_visual_info.depth, InputOutput,
                                     xlib_visual_info.visual, 
                                     CWEventMask, &xlib_window_attr);

  uint bytes_per_pixel = 4;
  AppPixelBuffer pixel_buffer = {0};
  pixel_buffer.width = window_width;
  pixel_buffer.height = window_height;
  pixel_buffer.pixels = calloc(pixel_buffer.width * pixel_buffer.height,
                             bytes_per_pixel);
  if (pixel_buffer.pixels == NULL)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    EBP();
  }

  int pixel_offset = 0;
  int pixel_pad = 32;
  int bytes_per_line = 0;
  XImage *xlib_image = XCreateImage(xlib_display, xlib_visual_info.visual,
                                   xlib_visual_info.depth, ZPixmap,
                                   pixel_offset, (u8 *)pixel_buffer.pixels, 
                                   pixel_buffer.width,
                                   pixel_buffer.height, pixel_pad, 
                                   bytes_per_line);
  if (xlib_image == NULL)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    BP();
  }

  Pixmap xlib_pixmap = XCreatePixmap(xlib_display, xlib_window,
                                      pixel_buffer.width, 
                                      pixel_buffer.height,
                                      xlib_visual_info.depth);

  XRenderPictFormat *xlib_pic_format = \
      XRenderFindVisualFormat(xlib_display, xlib_visual_info.visual);
  if (xlib_pic_format == NULL)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    BP();
  }

  XStoreName(xlib_display, xlib_window, "APP");
  XMapWindow(xlib_display, xlib_window);

  Atom xlib_delete_window = XInternAtom(xlib_display, 
                                   "WM_DELETE_WINDOW", False);
  Status wm_set_res = XSetWMProtocols(xlib_display, xlib_window, 
                                      &xlib_delete_window, 1);
  if (wm_set_res == False)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    BP();
  }

  XFlush(xlib_display);

  int evdev_epoll_fd = epoll_create1(0);
  if (evdev_get_input_devices(evdev_epoll_fd) == -1)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    BP();
  }

  uint refresh_rate = xrandr_get_active_refresh_rate(xlib_display, 
                                                     xlib_root_window);
  uint app_refresh_rate = (r32)refresh_rate / 2.0f;
  r32 nsecs_per_frame = 1000000.0f / (r32)app_refresh_rate;

  struct timespec start_ts = {0};
  // NOTE(Ryan): This clock is not subject to NTP
  if (clock_gettime(CLOCK_MONOTONIC_RAW, &start_ts) == -1)
  {
    // TODO(EHANDLING | ELOGGING: Ryan)
    EBP();
  }

  bool want_to_run = true;
  while (want_to_run)
  {{ CREATE_TIMED_BLOCK("program");
    XEvent xlib_event = {0}; 

    while (XCheckWindowEvent(xlib_display, xlib_window, 
                              StructureNotifyMask, &xlib_event))
    {
      if (xlib_event.type == ConfigureNotify)
      {
        window_width = xlib_event.xconfigure.width;
        window_height = xlib_event.xconfigure.height;
      }
    }

    while (XCheckTypedWindowEvent(xlib_display, xlib_window, 
                                  ClientMessage, &xlib_event))
    {
      if (xlib_event.xclient.data.l[0] == xlib_delete_window)
      {
        XDestroyWindow(xlib_display, xlib_window);
        want_to_run = false;
      }
    }

    // TODO(PROD | Ryan): Use netlink socket for hotplugging 
#define MAX_EVENTS 5
#define TIMEOUT_MS 1
    // TODO(SPEED | Ryan): Using epoll has introduced noticeable lag?
    struct epoll_event evdev_epoll_events[MAX_EVENTS] = {0};
    uint num_evdev_epoll_events = 0;
    {
    CREATE_TIMED_BLOCK("evdev_wait");
    num_evdev_epoll_events = epoll_wait(evdev_epoll_fd, 
                                        evdev_epoll_events, 
                                        MAX_EVENTS, TIMEOUT_MS);
    }

    for (uint evdev_epoll_event = 0;
        evdev_epoll_event < num_evdev_epoll_events; 
        ++evdev_epoll_event)
    {
      struct input_event events[32] = {0};
      int len = read(evdev_epoll_events[evdev_epoll_event].data.fd, 
                     events, sizeof(events));
      if (len == -1)
      {
        // TODO(EHANDLING | ELOGGING: Ryan)
        EBP();
      }
      for (uint event_i = 0; event_i < len / sizeof(events[0]); ++event_i)
      {
        switch (events[event_i].type)
        {
          case EV_KEY:
          {
            if (events[event_i].code == KEY_W)
            {
              puts("entered W");
            }
            bool is_released = (events[event_i].value == 0);
          } break;
        }
      }
    }

    app_render(&pixel_buffer);

    // TODO(PROD | Ryan): Switch to glx to ensure vsynced rendering
    xrender_display_pixel_buffer(xlib_display, xlib_pic_format, 
                                 xlib_pixmap, xlib_image, xlib_window, 
                                 window_width, window_height);
#if 0
    struct timespec end_ts = {0};
    if (clock_gettime(CLOCK_MONOTONIC_RAW, &end_ts) == -1)
    {
      // TODO(EHANDLING | ELOGGING: Ryan)
      EBP();
    }

    long nsecs_elapsed = end_ts.tv_nsec - start_ts.tv_nsec;
    if (nsecs_elapsed < nsecs_per_frame)
    {
      struct timespec sleep_ts, elapsed_ts = {0};
      long nsecs_to_sleep = nsecs_per_frame - nsecs_elapsed;
      elapsed_ts.tv_sec = nsecs_to_sleep / 1000000.0f;
      elapsed_ts.tv_nsec = (nsecs_to_sleep % 1000000) * 1000000;
      int sleep_status = 0;
      do
      {
        sleep_ts = elapsed_ts;
        sleep_status = nanosleep(&sleep_ts, &elapsed_ts);
      } while (sleep_status == -1 && errno == EINTR);
    }

    start_ts = end_ts;
#endif
    // NOTE(Ryan): \033 is ANSI escape code for ESC. This has value 27, 
    // i.e. 0x1B. This is followed by [. Together they mark a 
    // Control Sequence Introducer. Arguments to this will follow.
    // printf("\x1B[" "2J");
    for (uint timed_block_num = 0; 
        timed_block_num < __COUNTER__;
        timed_block_num++)
    {
      TimedBlock timed_block = timed_blocks[timed_block_num];
      printf("(%d) %s: %fms\n", timed_block_num, timed_block.name, 
             timed_block.time_ms / 1000.0f);
    }
  }}

  return 0;
}

TimedBlock timed_blocks[__COUNTER__ - 1] = {0};
