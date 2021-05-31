// SPDX-License-Identifier: zlib-acknowledgement
#pragma once

#include "app.h"

INTERNAL void
app_render(AppPixelBuffer *pixel_buffer)
{
  for (uint y = 0; y < pixel_buffer->height; ++y)
  {
    for (uint x = 0; x < pixel_buffer->width; ++x)
    {
      u8 red = 253;
      u8 green = 232;
      u8 blue = 227;
      pixel_buffer->pixels[y * pixel_buffer->width + x] = \
          (red << 16 | green << 8 | blue);
    }
  }
}
