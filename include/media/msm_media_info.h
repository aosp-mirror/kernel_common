#ifndef __MEDIA_INFO_H__
#define __MEDIA_INFO_H__

#ifndef MSM_MEDIA_ALIGN
#define MSM_MEDIA_ALIGN(__sz, __align) (((__sz) + (__align-1)) & (~(__align-1)))
#endif

enum color_fmts {
	/* Venus NV12:
	 * YUV 4:2:0 image with a plane of 8 bit Y samples followed
	 * by an interleaved U/V plane containing 8 bit 2x2 subsampled
	 * colour difference samples.
	 *
	 * <-------- Y/UV_Stride -------->
	 * <------- Width ------->
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  ^           ^
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |           |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  Height      |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |          Y_Scanlines
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |           |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |           |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |           |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  V           |
	 * X X X X X X X X X X X X X X X X              |
	 * X X X X X X X X X X X X X X X X              |
	 * X X X X X X X X X X X X X X X X              |
	 * X X X X X X X X X X X X X X X X              V
	 * U V U V U V U V U V U V X X X X  ^
	 * U V U V U V U V U V U V X X X X  |
	 * U V U V U V U V U V U V X X X X  |
	 * U V U V U V U V U V U V X X X X  UV_Scanlines
	 * X X X X X X X X X X X X X X X X  |
	 * X X X X X X X X X X X X X X X X  V
	 * X X X X X X X X X X X X X X X X  --> Buffer size alignment
	 *
	 * Y_Stride : Width aligned to 128
	 * UV_Stride : Width aligned to 128
	 * Y_Scanlines: Height aligned to 32
	 * UV_Scanlines: Height/2 aligned to 16
	 * Total size = align((Y_Stride * Y_Scanlines
	 *          + UV_Stride * UV_Scanlines + 4096), 4096)
	 */
	COLOR_FMT_NV12,

	/* Venus NV21:
	 * YUV 4:2:0 image with a plane of 8 bit Y samples followed
	 * by an interleaved V/U plane containing 8 bit 2x2 subsampled
	 * colour difference samples.
	 *
	 * <-------- Y/UV_Stride -------->
	 * <------- Width ------->
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  ^           ^
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |           |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  Height      |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |          Y_Scanlines
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |           |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |           |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  |           |
	 * Y Y Y Y Y Y Y Y Y Y Y Y X X X X  V           |
	 * X X X X X X X X X X X X X X X X              |
	 * X X X X X X X X X X X X X X X X              |
	 * X X X X X X X X X X X X X X X X              |
	 * X X X X X X X X X X X X X X X X              V
	 * V U V U V U V U V U V U X X X X  ^
	 * V U V U V U V U V U V U X X X X  |
	 * V U V U V U V U V U V U X X X X  |
	 * V U V U V U V U V U V U X X X X  UV_Scanlines
	 * X X X X X X X X X X X X X X X X  |
	 * X X X X X X X X X X X X X X X X  V
	 * X X X X X X X X X X X X X X X X  --> Padding & Buffer size alignment
	 *
	 * Y_Stride : Width aligned to 128
	 * UV_Stride : Width aligned to 128
	 * Y_Scanlines: Height aligned to 32
	 * UV_Scanlines: Height/2 aligned to 16
	 * Total size = align((Y_Stride * Y_Scanlines
	 *          + UV_Stride * UV_Scanlines + 4096), 4096)
	 */
	COLOR_FMT_NV21,
};

static inline unsigned int VENUS_EXTRADATA_SIZE(int width, int height)
{
	(void)height;
	(void)width;

	/*
	 * In the future, calculate the size based on the w/h but just
	 * hardcode it for now since 8K satisfies all current usecases.
	 */
	return 8 * 1024;
}

static inline unsigned int VENUS_Y_STRIDE(int color_fmt, int width)
{
	unsigned int alignment, stride = 0;
	if (!width)
		goto invalid_input;

	switch (color_fmt) {
	case COLOR_FMT_NV21:
	case COLOR_FMT_NV12:
		alignment = 128;
		stride = MSM_MEDIA_ALIGN(width, alignment);
		break;
	default:
		break;
	}
invalid_input:
	return stride;
}

static inline unsigned int VENUS_UV_STRIDE(int color_fmt, int width)
{
	unsigned int alignment, stride = 0;
	if (!width)
		goto invalid_input;

	switch (color_fmt) {
	case COLOR_FMT_NV21:
	case COLOR_FMT_NV12:
		alignment = 128;
		stride = MSM_MEDIA_ALIGN(width, alignment);
		break;
	default:
		break;
	}
invalid_input:
	return stride;
}

static inline unsigned int VENUS_Y_SCANLINES(int color_fmt, int height)
{
	unsigned int alignment, sclines = 0;
	if (!height)
		goto invalid_input;

	switch (color_fmt) {
	case COLOR_FMT_NV21:
	case COLOR_FMT_NV12:
		alignment = 32;
		sclines = MSM_MEDIA_ALIGN(height, alignment);
		break;
	default:
		break;
	}
invalid_input:
	return sclines;
}

static inline unsigned int VENUS_UV_SCANLINES(int color_fmt, int height)
{
	unsigned int alignment, sclines = 0;
	if (!height)
		goto invalid_input;

	switch (color_fmt) {
	case COLOR_FMT_NV21:
	case COLOR_FMT_NV12:
		alignment = 16;
		sclines = MSM_MEDIA_ALIGN(((height + 1) >> 1), alignment);
		break;
	default:
		break;
	}
invalid_input:
	return sclines;
}

static inline unsigned int VENUS_BUFFER_SIZE(
	int color_fmt, int width, int height)
{
	const unsigned int extra_size = VENUS_EXTRADATA_SIZE(width, height);
	unsigned int uv_alignment = 0, size = 0;
	unsigned int y_plane, uv_plane, y_stride,
		uv_stride, y_sclines, uv_sclines;
	if (!width || !height)
		goto invalid_input;

	y_stride = VENUS_Y_STRIDE(color_fmt, width);
	uv_stride = VENUS_UV_STRIDE(color_fmt, width);
	y_sclines = VENUS_Y_SCANLINES(color_fmt, height);
	uv_sclines = VENUS_UV_SCANLINES(color_fmt, height);
	switch (color_fmt) {
	case COLOR_FMT_NV21:
	case COLOR_FMT_NV12:
		uv_alignment = 4096;
		y_plane = y_stride * y_sclines;
		uv_plane = uv_stride * uv_sclines + uv_alignment;
		size = y_plane + uv_plane + extra_size;
		size = MSM_MEDIA_ALIGN(size, 4096);
		break;
	default:
		break;
	}
invalid_input:
	return size;
}

#endif
