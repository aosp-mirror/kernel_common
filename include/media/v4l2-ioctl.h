#ifndef _V4L2_IOCTL_H
#define _V4L2_IOCTL_H

#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/compiler.h> 
#include <linux/videodev2.h>

struct v4l2_fh;

struct v4l2_ioctl_ops {
	

	
	int (*vidioc_querycap)(struct file *file, void *fh, struct v4l2_capability *cap);

	
	int (*vidioc_g_priority)   (struct file *file, void *fh,
				    enum v4l2_priority *p);
	int (*vidioc_s_priority)   (struct file *file, void *fh,
				    enum v4l2_priority p);

	
	int (*vidioc_enum_fmt_vid_cap)     (struct file *file, void *fh,
					    struct v4l2_fmtdesc *f);
	int (*vidioc_enum_fmt_vid_overlay) (struct file *file, void *fh,
					    struct v4l2_fmtdesc *f);
	int (*vidioc_enum_fmt_vid_out)     (struct file *file, void *fh,
					    struct v4l2_fmtdesc *f);
	int (*vidioc_enum_fmt_vid_cap_mplane)(struct file *file, void *fh,
					      struct v4l2_fmtdesc *f);
	int (*vidioc_enum_fmt_vid_out_mplane)(struct file *file, void *fh,
					      struct v4l2_fmtdesc *f);
	int (*vidioc_enum_fmt_type_private)(struct file *file, void *fh,
					    struct v4l2_fmtdesc *f);

	
	int (*vidioc_g_fmt_vid_cap)    (struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_g_fmt_vid_overlay)(struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_g_fmt_vid_out)    (struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_g_fmt_vid_out_overlay)(struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_g_fmt_vbi_cap)    (struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_g_fmt_vbi_out)    (struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_g_fmt_sliced_vbi_cap)(struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_g_fmt_sliced_vbi_out)(struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_g_fmt_vid_cap_mplane)(struct file *file, void *fh,
					   struct v4l2_format *f);
	int (*vidioc_g_fmt_vid_out_mplane)(struct file *file, void *fh,
					   struct v4l2_format *f);
	int (*vidioc_g_fmt_type_private)(struct file *file, void *fh,
					struct v4l2_format *f);

	
	int (*vidioc_s_fmt_vid_cap)    (struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_s_fmt_vid_overlay)(struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_s_fmt_vid_out)    (struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_s_fmt_vid_out_overlay)(struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_s_fmt_vbi_cap)    (struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_s_fmt_vbi_out)    (struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_s_fmt_sliced_vbi_cap)(struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_s_fmt_sliced_vbi_out)(struct file *file, void *fh,
					struct v4l2_format *f);
	int (*vidioc_s_fmt_vid_cap_mplane)(struct file *file, void *fh,
					   struct v4l2_format *f);
	int (*vidioc_s_fmt_vid_out_mplane)(struct file *file, void *fh,
					   struct v4l2_format *f);
	int (*vidioc_s_fmt_type_private)(struct file *file, void *fh,
					struct v4l2_format *f);

	
	int (*vidioc_try_fmt_vid_cap)    (struct file *file, void *fh,
					  struct v4l2_format *f);
	int (*vidioc_try_fmt_vid_overlay)(struct file *file, void *fh,
					  struct v4l2_format *f);
	int (*vidioc_try_fmt_vid_out)    (struct file *file, void *fh,
					  struct v4l2_format *f);
	int (*vidioc_try_fmt_vid_out_overlay)(struct file *file, void *fh,
					  struct v4l2_format *f);
	int (*vidioc_try_fmt_vbi_cap)    (struct file *file, void *fh,
					  struct v4l2_format *f);
	int (*vidioc_try_fmt_vbi_out)    (struct file *file, void *fh,
					  struct v4l2_format *f);
	int (*vidioc_try_fmt_sliced_vbi_cap)(struct file *file, void *fh,
					  struct v4l2_format *f);
	int (*vidioc_try_fmt_sliced_vbi_out)(struct file *file, void *fh,
					  struct v4l2_format *f);
	int (*vidioc_try_fmt_vid_cap_mplane)(struct file *file, void *fh,
					     struct v4l2_format *f);
	int (*vidioc_try_fmt_vid_out_mplane)(struct file *file, void *fh,
					     struct v4l2_format *f);
	int (*vidioc_try_fmt_type_private)(struct file *file, void *fh,
					  struct v4l2_format *f);

	
	int (*vidioc_reqbufs) (struct file *file, void *fh, struct v4l2_requestbuffers *b);
	int (*vidioc_querybuf)(struct file *file, void *fh, struct v4l2_buffer *b);
	int (*vidioc_qbuf)    (struct file *file, void *fh, struct v4l2_buffer *b);
	int (*vidioc_dqbuf)   (struct file *file, void *fh, struct v4l2_buffer *b);

	int (*vidioc_create_bufs)(struct file *file, void *fh, struct v4l2_create_buffers *b);
	int (*vidioc_prepare_buf)(struct file *file, void *fh, struct v4l2_buffer *b);

	int (*vidioc_overlay) (struct file *file, void *fh, unsigned int i);
	int (*vidioc_g_fbuf)   (struct file *file, void *fh,
				struct v4l2_framebuffer *a);
	int (*vidioc_s_fbuf)   (struct file *file, void *fh,
				struct v4l2_framebuffer *a);

		
	int (*vidioc_streamon) (struct file *file, void *fh, enum v4l2_buf_type i);
	int (*vidioc_streamoff)(struct file *file, void *fh, enum v4l2_buf_type i);

	int (*vidioc_g_std) (struct file *file, void *fh, v4l2_std_id *norm);
	int (*vidioc_s_std) (struct file *file, void *fh, v4l2_std_id *norm);
	int (*vidioc_querystd) (struct file *file, void *fh, v4l2_std_id *a);

		
	int (*vidioc_enum_input)(struct file *file, void *fh,
				 struct v4l2_input *inp);
	int (*vidioc_g_input)   (struct file *file, void *fh, unsigned int *i);
	int (*vidioc_s_input)   (struct file *file, void *fh, unsigned int i);

		
	int (*vidioc_enum_output) (struct file *file, void *fh,
				  struct v4l2_output *a);
	int (*vidioc_g_output)   (struct file *file, void *fh, unsigned int *i);
	int (*vidioc_s_output)   (struct file *file, void *fh, unsigned int i);

		
	int (*vidioc_queryctrl)        (struct file *file, void *fh,
					struct v4l2_queryctrl *a);
	int (*vidioc_g_ctrl)           (struct file *file, void *fh,
					struct v4l2_control *a);
	int (*vidioc_s_ctrl)           (struct file *file, void *fh,
					struct v4l2_control *a);
	int (*vidioc_g_ext_ctrls)      (struct file *file, void *fh,
					struct v4l2_ext_controls *a);
	int (*vidioc_s_ext_ctrls)      (struct file *file, void *fh,
					struct v4l2_ext_controls *a);
	int (*vidioc_try_ext_ctrls)    (struct file *file, void *fh,
					struct v4l2_ext_controls *a);
	int (*vidioc_querymenu)        (struct file *file, void *fh,
					struct v4l2_querymenu *a);

	
	int (*vidioc_enumaudio)        (struct file *file, void *fh,
					struct v4l2_audio *a);
	int (*vidioc_g_audio)          (struct file *file, void *fh,
					struct v4l2_audio *a);
	int (*vidioc_s_audio)          (struct file *file, void *fh,
					struct v4l2_audio *a);

	
	int (*vidioc_enumaudout)       (struct file *file, void *fh,
					struct v4l2_audioout *a);
	int (*vidioc_g_audout)         (struct file *file, void *fh,
					struct v4l2_audioout *a);
	int (*vidioc_s_audout)         (struct file *file, void *fh,
					struct v4l2_audioout *a);
	int (*vidioc_g_modulator)      (struct file *file, void *fh,
					struct v4l2_modulator *a);
	int (*vidioc_s_modulator)      (struct file *file, void *fh,
					struct v4l2_modulator *a);
	
	int (*vidioc_cropcap)          (struct file *file, void *fh,
					struct v4l2_cropcap *a);
	int (*vidioc_g_crop)           (struct file *file, void *fh,
					struct v4l2_crop *a);
	int (*vidioc_s_crop)           (struct file *file, void *fh,
					struct v4l2_crop *a);
	int (*vidioc_g_selection)      (struct file *file, void *fh,
					struct v4l2_selection *s);
	int (*vidioc_s_selection)      (struct file *file, void *fh,
					struct v4l2_selection *s);
	
	int (*vidioc_g_jpegcomp)       (struct file *file, void *fh,
					struct v4l2_jpegcompression *a);
	int (*vidioc_s_jpegcomp)       (struct file *file, void *fh,
					struct v4l2_jpegcompression *a);
	int (*vidioc_g_enc_index)      (struct file *file, void *fh,
					struct v4l2_enc_idx *a);
	int (*vidioc_encoder_cmd)      (struct file *file, void *fh,
					struct v4l2_encoder_cmd *a);
	int (*vidioc_try_encoder_cmd)  (struct file *file, void *fh,
					struct v4l2_encoder_cmd *a);
	int (*vidioc_decoder_cmd)      (struct file *file, void *fh,
					struct v4l2_decoder_cmd *a);
	int (*vidioc_try_decoder_cmd)  (struct file *file, void *fh,
					struct v4l2_decoder_cmd *a);

	
	int (*vidioc_g_parm)           (struct file *file, void *fh,
					struct v4l2_streamparm *a);
	int (*vidioc_s_parm)           (struct file *file, void *fh,
					struct v4l2_streamparm *a);

	
	int (*vidioc_g_tuner)          (struct file *file, void *fh,
					struct v4l2_tuner *a);
	int (*vidioc_s_tuner)          (struct file *file, void *fh,
					struct v4l2_tuner *a);
	int (*vidioc_g_frequency)      (struct file *file, void *fh,
					struct v4l2_frequency *a);
	int (*vidioc_s_frequency)      (struct file *file, void *fh,
					struct v4l2_frequency *a);

	
	int (*vidioc_g_sliced_vbi_cap) (struct file *file, void *fh,
					struct v4l2_sliced_vbi_cap *a);

	
	int (*vidioc_log_status)       (struct file *file, void *fh);

	int (*vidioc_s_hw_freq_seek)   (struct file *file, void *fh,
					struct v4l2_hw_freq_seek *a);

	
#ifdef CONFIG_VIDEO_ADV_DEBUG
	int (*vidioc_g_register)       (struct file *file, void *fh,
					struct v4l2_dbg_register *reg);
	int (*vidioc_s_register)       (struct file *file, void *fh,
					struct v4l2_dbg_register *reg);
#endif
	int (*vidioc_g_chip_ident)     (struct file *file, void *fh,
					struct v4l2_dbg_chip_ident *chip);

	int (*vidioc_enum_framesizes)   (struct file *file, void *fh,
					 struct v4l2_frmsizeenum *fsize);

	int (*vidioc_enum_frameintervals) (struct file *file, void *fh,
					   struct v4l2_frmivalenum *fival);

	
	int (*vidioc_enum_dv_presets) (struct file *file, void *fh,
				       struct v4l2_dv_enum_preset *preset);

	int (*vidioc_s_dv_preset) (struct file *file, void *fh,
				   struct v4l2_dv_preset *preset);
	int (*vidioc_g_dv_preset) (struct file *file, void *fh,
				   struct v4l2_dv_preset *preset);
	int (*vidioc_query_dv_preset) (struct file *file, void *fh,
					struct v4l2_dv_preset *qpreset);
	int (*vidioc_s_dv_timings) (struct file *file, void *fh,
				    struct v4l2_dv_timings *timings);
	int (*vidioc_g_dv_timings) (struct file *file, void *fh,
				    struct v4l2_dv_timings *timings);

	int (*vidioc_subscribe_event)  (struct v4l2_fh *fh,
					struct v4l2_event_subscription *sub);
	int (*vidioc_unsubscribe_event)(struct v4l2_fh *fh,
					struct v4l2_event_subscription *sub);

	
	long (*vidioc_default)	       (struct file *file, void *fh,
					bool valid_prio, int cmd, void *arg);

	
	int (*vidioc_htc_set_callingpid_name)(struct file *file, void *fh, struct htc_callingpid_data *b);
	
};



#define V4L2_DEBUG_IOCTL     0x01
#define V4L2_DEBUG_IOCTL_ARG 0x02

#define v4l_print_ioctl(name, cmd)  		 \
	do {  					 \
		printk(KERN_DEBUG "%s: ", name); \
		v4l_printk_ioctl(cmd);		 \
	} while (0)

#define v4l_i2c_print_ioctl(client, cmd) 		   \
	do {      					   \
		v4l_client_printk(KERN_DEBUG, client, ""); \
		v4l_printk_ioctl(cmd);			   \
	} while (0)

extern const char *v4l2_norm_to_name(v4l2_std_id id);
extern void v4l2_video_std_frame_period(int id, struct v4l2_fract *frameperiod);
extern int v4l2_video_std_construct(struct v4l2_standard *vs,
				    int id, const char *name);
extern void v4l_printk_ioctl(unsigned int cmd);

extern const char *v4l2_field_names[];
extern const char *v4l2_type_names[];

#ifdef CONFIG_COMPAT
extern long v4l2_compat_ioctl32(struct file *file, unsigned int cmd,
				unsigned long arg);
#endif

typedef long (*v4l2_kioctl)(struct file *file,
		unsigned int cmd, void *arg);

extern long video_usercopy(struct file *file, unsigned int cmd,
				unsigned long arg, v4l2_kioctl func);

extern long video_ioctl2(struct file *file,
			unsigned int cmd, unsigned long arg);

#endif 
