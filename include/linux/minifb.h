#ifndef _MINI_FB_H_
#define _MINI_FB_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define MINIFB_IOCTL_MAGIC 'H'

#define MINIFB_INIT _IOW(MINIFB_IOCTL_MAGIC, 1, struct minifb_session)
#define MINIFB_TERMINATE _IOW(MINIFB_IOCTL_MAGIC, 2, struct minifb_session)
#define MINIFB_QUEUE_BUFFER _IOW(MINIFB_IOCTL_MAGIC, 3, struct minifb_req)
#define MINIFB_DEQUEUE_BUFFER _IOW(MINIFB_IOCTL_MAGIC, 4, struct minifb_req)

struct minifb_session {
	uint32_t token; /* reserve */
	uint32_t width;
	uint32_t height;
};

struct minifb_req {
	int token;
	int memory_id;
	uint32_t offset;
	uint32_t flags;
};

#ifdef __KERNEL__
#ifdef CONFIG_FB_MINIFB

#define MINIFB_NOREPEAT 0
#define MINIFB_REPEAT   1

int minifb_init(struct minifb_session *sess);
int minifb_terminate(struct minifb_session *sess);
int minifb_queuebuf(struct minifb_req *data);
int minifb_dequeuebuf(struct minifb_req *data);
int minifb_lockbuf(void **, unsigned long *, int);
void minifb_unlockbuf(void);
int minifb_ioctl_handler(unsigned int cmd, void *argp);
#else
int minifb_init(void)
{
	return -ENODEV;
}

int minifb_terminate(void)
{
	return -ENODEV;
}

int minifb_queuebuf(struct minifb_req *data)
{
	return -ENODEV;
}

int minifb_dequeuebuf(struct minifb_req *data)
{
	return -ENODEV;
}

int minifb_lockbuf(void **, unsigned long *)
{
	return -ENODEV;
}

void minifb_unlockbuf(void);
{

}

int minifb_ioctl_handler(unsigned int cmd, void *argp)
{
	return -ENODEV;
}
#endif
#endif
#endif /* _MINI_FB_H_ */
