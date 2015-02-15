/*
 * n_tty.c --- implements the N_TTY line discipline.
 *
 * This code used to be in tty_io.c, but things are getting hairy
 * enough that it made sense to split things off.  (The N_TTY
 * processing has changed so much that it's hardly recognizable,
 * anyway...)
 *
 * Note that the open routine for N_TTY is guaranteed never to return
 * an error.  This is because Linux will fall back to setting a line
 * to N_TTY if it can not switch to any other line discipline.
 *
 * Written by Theodore Ts'o, Copyright 1994.
 *
 * This file also contains code originally written by Linus Torvalds,
 * Copyright 1991, 1992, 1993, and by Julian Cowley, Copyright 1994.
 *
 * This file may be redistributed under the terms of the GNU General Public
 * License.
 *
 * Reduced memory usage for older ARM systems  - Russell King.
 *
 * 2000/01/20   Fixed SMP locking on put_tty_queue using bits of
 *		the patch by Andrew J. Kroll <ag784@freenet.buffalo.edu>
 *		who actually finally proved there really was a race.
 *
 * 2002/03/18   Implemented n_tty_wakeup to send SIGIO POLL_OUTs to
 *		waiting writing processes-Sapan Bhatia <sapan@corewars.org>.
 *		Also fixed a bug in BLOCKING mode where n_tty_write returns
 *		EAGAIN
 */

#include <linux/types.h>
#include <linux/major.h>
#include <linux/errno.h>
#include <linux/signal.h>
#include <linux/fcntl.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/timer.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/bitops.h>
#include <linux/audit.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/module.h>


#define WAKEUP_CHARS 256

#define TTY_THRESHOLD_THROTTLE		128 
#define TTY_THRESHOLD_UNTHROTTLE	128

#define ECHO_OP_START 0xff
#define ECHO_OP_MOVE_BACK_COL 0x80
#define ECHO_OP_SET_CANON_COL 0x81
#define ECHO_OP_ERASE_TAB 0x82

static inline int tty_put_user(struct tty_struct *tty, unsigned char x,
			       unsigned char __user *ptr)
{
	tty_audit_add_data(tty, &x, 1);
	return put_user(x, ptr);
}


static void n_tty_set_room(struct tty_struct *tty)
{
	int left;
	int old_left;
	unsigned long flags;

	spin_lock_irqsave(&tty->read_lock, flags);

	if (I_PARMRK(tty)) {
		left = N_TTY_BUF_SIZE - tty->read_cnt * 3 - 1;
	} else
		left = N_TTY_BUF_SIZE - tty->read_cnt - 1;

	if (left <= 0)
		left = tty->icanon && !tty->canon_data;
	old_left = tty->receive_room;
	tty->receive_room = left;

	spin_unlock_irqrestore(&tty->read_lock, flags);

	
	if (left && !old_left)
		schedule_work(&tty->buf.work);
}

static void put_tty_queue_nolock(unsigned char c, struct tty_struct *tty)
{
	if (tty->read_cnt < N_TTY_BUF_SIZE) {
		tty->read_buf[tty->read_head] = c;
		tty->read_head = (tty->read_head + 1) & (N_TTY_BUF_SIZE-1);
		tty->read_cnt++;
	}
}


static void put_tty_queue(unsigned char c, struct tty_struct *tty)
{
	unsigned long flags;
	spin_lock_irqsave(&tty->read_lock, flags);
	put_tty_queue_nolock(c, tty);
	spin_unlock_irqrestore(&tty->read_lock, flags);
}

static void check_unthrottle(struct tty_struct *tty)
{
	if (tty->count)
		tty_unthrottle(tty);
}


static void reset_buffer_flags(struct tty_struct *tty)
{
	unsigned long flags;

	spin_lock_irqsave(&tty->read_lock, flags);
	tty->read_head = tty->read_tail = tty->read_cnt = 0;
	spin_unlock_irqrestore(&tty->read_lock, flags);

	mutex_lock(&tty->echo_lock);
	tty->echo_pos = tty->echo_cnt = tty->echo_overrun = 0;
	mutex_unlock(&tty->echo_lock);

	tty->canon_head = tty->canon_data = tty->erasing = 0;
	memset(&tty->read_flags, 0, sizeof tty->read_flags);
	n_tty_set_room(tty);
}


static void n_tty_flush_buffer(struct tty_struct *tty)
{
	unsigned long flags;
	
	reset_buffer_flags(tty);

	if (!tty->link)
		return;

	spin_lock_irqsave(&tty->ctrl_lock, flags);
	if (tty->link->packet) {
		tty->ctrl_status |= TIOCPKT_FLUSHREAD;
		wake_up_interruptible(&tty->link->read_wait);
	}
	spin_unlock_irqrestore(&tty->ctrl_lock, flags);
}


static ssize_t n_tty_chars_in_buffer(struct tty_struct *tty)
{
	unsigned long flags;
	ssize_t n = 0;

	spin_lock_irqsave(&tty->read_lock, flags);
	if (!tty->icanon) {
		n = tty->read_cnt;
	} else if (tty->canon_data) {
		n = (tty->canon_head > tty->read_tail) ?
			tty->canon_head - tty->read_tail :
			tty->canon_head + (N_TTY_BUF_SIZE - tty->read_tail);
	}
	spin_unlock_irqrestore(&tty->read_lock, flags);
	return n;
}


static inline int is_utf8_continuation(unsigned char c)
{
	return (c & 0xc0) == 0x80;
}


static inline int is_continuation(unsigned char c, struct tty_struct *tty)
{
	return I_IUTF8(tty) && is_utf8_continuation(c);
}


static int do_output_char(unsigned char c, struct tty_struct *tty, int space)
{
	int	spaces;

	if (!space)
		return -1;

	switch (c) {
	case '\n':
		if (O_ONLRET(tty))
			tty->column = 0;
		if (O_ONLCR(tty)) {
			if (space < 2)
				return -1;
			tty->canon_column = tty->column = 0;
			tty->ops->write(tty, "\r\n", 2);
			return 2;
		}
		tty->canon_column = tty->column;
		break;
	case '\r':
		if (O_ONOCR(tty) && tty->column == 0)
			return 0;
		if (O_OCRNL(tty)) {
			c = '\n';
			if (O_ONLRET(tty))
				tty->canon_column = tty->column = 0;
			break;
		}
		tty->canon_column = tty->column = 0;
		break;
	case '\t':
		spaces = 8 - (tty->column & 7);
		if (O_TABDLY(tty) == XTABS) {
			if (space < spaces)
				return -1;
			tty->column += spaces;
			tty->ops->write(tty, "        ", spaces);
			return spaces;
		}
		tty->column += spaces;
		break;
	case '\b':
		if (tty->column > 0)
			tty->column--;
		break;
	default:
		if (!iscntrl(c)) {
			if (O_OLCUC(tty))
				c = toupper(c);
			if (!is_continuation(c, tty))
				tty->column++;
		}
		break;
	}

	tty_put_char(tty, c);
	return 1;
}


static int process_output(unsigned char c, struct tty_struct *tty)
{
	int	space, retval;

	mutex_lock(&tty->output_lock);

	space = tty_write_room(tty);
	retval = do_output_char(c, tty, space);

	mutex_unlock(&tty->output_lock);
	if (retval < 0)
		return -1;
	else
		return 0;
}


static ssize_t process_output_block(struct tty_struct *tty,
				    const unsigned char *buf, unsigned int nr)
{
	int	space;
	int	i;
	const unsigned char *cp;

	mutex_lock(&tty->output_lock);

	space = tty_write_room(tty);
	if (!space) {
		mutex_unlock(&tty->output_lock);
		return 0;
	}
	if (nr > space)
		nr = space;

	for (i = 0, cp = buf; i < nr; i++, cp++) {
		unsigned char c = *cp;

		switch (c) {
		case '\n':
			if (O_ONLRET(tty))
				tty->column = 0;
			if (O_ONLCR(tty))
				goto break_out;
			tty->canon_column = tty->column;
			break;
		case '\r':
			if (O_ONOCR(tty) && tty->column == 0)
				goto break_out;
			if (O_OCRNL(tty))
				goto break_out;
			tty->canon_column = tty->column = 0;
			break;
		case '\t':
			goto break_out;
		case '\b':
			if (tty->column > 0)
				tty->column--;
			break;
		default:
			if (!iscntrl(c)) {
				if (O_OLCUC(tty))
					goto break_out;
				if (!is_continuation(c, tty))
					tty->column++;
			}
			break;
		}
	}
break_out:
	i = tty->ops->write(tty, buf, i);

	mutex_unlock(&tty->output_lock);
	return i;
}


static void process_echoes(struct tty_struct *tty)
{
	int	space, nr;
	unsigned char c;
	unsigned char *cp, *buf_end;

	if (!tty->echo_cnt)
		return;

	mutex_lock(&tty->output_lock);
	mutex_lock(&tty->echo_lock);

	space = tty_write_room(tty);

	buf_end = tty->echo_buf + N_TTY_BUF_SIZE;
	cp = tty->echo_buf + tty->echo_pos;
	nr = tty->echo_cnt;
	while (nr > 0) {
		c = *cp;
		if (c == ECHO_OP_START) {
			unsigned char op;
			unsigned char *opp;
			int no_space_left = 0;

			opp = cp + 1;
			if (opp == buf_end)
				opp -= N_TTY_BUF_SIZE;
			op = *opp;

			switch (op) {
				unsigned int num_chars, num_bs;

			case ECHO_OP_ERASE_TAB:
				if (++opp == buf_end)
					opp -= N_TTY_BUF_SIZE;
				num_chars = *opp;

				if (!(num_chars & 0x80))
					num_chars += tty->canon_column;
				num_bs = 8 - (num_chars & 7);

				if (num_bs > space) {
					no_space_left = 1;
					break;
				}
				space -= num_bs;
				while (num_bs--) {
					tty_put_char(tty, '\b');
					if (tty->column > 0)
						tty->column--;
				}
				cp += 3;
				nr -= 3;
				break;

			case ECHO_OP_SET_CANON_COL:
				tty->canon_column = tty->column;
				cp += 2;
				nr -= 2;
				break;

			case ECHO_OP_MOVE_BACK_COL:
				if (tty->column > 0)
					tty->column--;
				cp += 2;
				nr -= 2;
				break;

			case ECHO_OP_START:
				
				if (!space) {
					no_space_left = 1;
					break;
				}
				tty_put_char(tty, ECHO_OP_START);
				tty->column++;
				space--;
				cp += 2;
				nr -= 2;
				break;

			default:
				if (space < 2) {
					no_space_left = 1;
					break;
				}
				tty_put_char(tty, '^');
				tty_put_char(tty, op ^ 0100);
				tty->column += 2;
				space -= 2;
				cp += 2;
				nr -= 2;
			}

			if (no_space_left)
				break;
		} else {
			if (O_OPOST(tty) &&
			    !(test_bit(TTY_HW_COOK_OUT, &tty->flags))) {
				int retval = do_output_char(c, tty, space);
				if (retval < 0)
					break;
				space -= retval;
			} else {
				if (!space)
					break;
				tty_put_char(tty, c);
				space -= 1;
			}
			cp += 1;
			nr -= 1;
		}

		
		if (cp >= buf_end)
			cp -= N_TTY_BUF_SIZE;
	}

	if (nr == 0) {
		tty->echo_pos = 0;
		tty->echo_cnt = 0;
		tty->echo_overrun = 0;
	} else {
		int num_processed = tty->echo_cnt - nr;
		tty->echo_pos += num_processed;
		tty->echo_pos &= N_TTY_BUF_SIZE - 1;
		tty->echo_cnt = nr;
		if (num_processed > 0)
			tty->echo_overrun = 0;
	}

	mutex_unlock(&tty->echo_lock);
	mutex_unlock(&tty->output_lock);

	if (tty->ops->flush_chars)
		tty->ops->flush_chars(tty);
}


static void add_echo_byte(unsigned char c, struct tty_struct *tty)
{
	int	new_byte_pos;

	if (tty->echo_cnt == N_TTY_BUF_SIZE) {
		
		new_byte_pos = tty->echo_pos;

		if (tty->echo_buf[tty->echo_pos] == ECHO_OP_START) {
			if (tty->echo_buf[(tty->echo_pos + 1) &
					  (N_TTY_BUF_SIZE - 1)] ==
						ECHO_OP_ERASE_TAB) {
				tty->echo_pos += 3;
				tty->echo_cnt -= 2;
			} else {
				tty->echo_pos += 2;
				tty->echo_cnt -= 1;
			}
		} else {
			tty->echo_pos++;
		}
		tty->echo_pos &= N_TTY_BUF_SIZE - 1;

		tty->echo_overrun = 1;
	} else {
		new_byte_pos = tty->echo_pos + tty->echo_cnt;
		new_byte_pos &= N_TTY_BUF_SIZE - 1;
		tty->echo_cnt++;
	}

	tty->echo_buf[new_byte_pos] = c;
}


static void echo_move_back_col(struct tty_struct *tty)
{
	mutex_lock(&tty->echo_lock);

	add_echo_byte(ECHO_OP_START, tty);
	add_echo_byte(ECHO_OP_MOVE_BACK_COL, tty);

	mutex_unlock(&tty->echo_lock);
}


static void echo_set_canon_col(struct tty_struct *tty)
{
	mutex_lock(&tty->echo_lock);

	add_echo_byte(ECHO_OP_START, tty);
	add_echo_byte(ECHO_OP_SET_CANON_COL, tty);

	mutex_unlock(&tty->echo_lock);
}


static void echo_erase_tab(unsigned int num_chars, int after_tab,
			   struct tty_struct *tty)
{
	mutex_lock(&tty->echo_lock);

	add_echo_byte(ECHO_OP_START, tty);
	add_echo_byte(ECHO_OP_ERASE_TAB, tty);

	
	num_chars &= 7;

	
	if (after_tab)
		num_chars |= 0x80;

	add_echo_byte(num_chars, tty);

	mutex_unlock(&tty->echo_lock);
}


static void echo_char_raw(unsigned char c, struct tty_struct *tty)
{
	mutex_lock(&tty->echo_lock);

	if (c == ECHO_OP_START) {
		add_echo_byte(ECHO_OP_START, tty);
		add_echo_byte(ECHO_OP_START, tty);
	} else {
		add_echo_byte(c, tty);
	}

	mutex_unlock(&tty->echo_lock);
}


static void echo_char(unsigned char c, struct tty_struct *tty)
{
	mutex_lock(&tty->echo_lock);

	if (c == ECHO_OP_START) {
		add_echo_byte(ECHO_OP_START, tty);
		add_echo_byte(ECHO_OP_START, tty);
	} else {
		if (L_ECHOCTL(tty) && iscntrl(c) && c != '\t')
			add_echo_byte(ECHO_OP_START, tty);
		add_echo_byte(c, tty);
	}

	mutex_unlock(&tty->echo_lock);
}


static inline void finish_erasing(struct tty_struct *tty)
{
	if (tty->erasing) {
		echo_char_raw('/', tty);
		tty->erasing = 0;
	}
}


static void eraser(unsigned char c, struct tty_struct *tty)
{
	enum { ERASE, WERASE, KILL } kill_type;
	int head, seen_alnums, cnt;
	unsigned long flags;

	
	if (tty->read_head == tty->canon_head) {
		 
		return;
	}
	if (c == ERASE_CHAR(tty))
		kill_type = ERASE;
	else if (c == WERASE_CHAR(tty))
		kill_type = WERASE;
	else {
		if (!L_ECHO(tty)) {
			spin_lock_irqsave(&tty->read_lock, flags);
			tty->read_cnt -= ((tty->read_head - tty->canon_head) &
					  (N_TTY_BUF_SIZE - 1));
			tty->read_head = tty->canon_head;
			spin_unlock_irqrestore(&tty->read_lock, flags);
			return;
		}
		if (!L_ECHOK(tty) || !L_ECHOKE(tty) || !L_ECHOE(tty)) {
			spin_lock_irqsave(&tty->read_lock, flags);
			tty->read_cnt -= ((tty->read_head - tty->canon_head) &
					  (N_TTY_BUF_SIZE - 1));
			tty->read_head = tty->canon_head;
			spin_unlock_irqrestore(&tty->read_lock, flags);
			finish_erasing(tty);
			echo_char(KILL_CHAR(tty), tty);
			
			if (L_ECHOK(tty))
				echo_char_raw('\n', tty);
			return;
		}
		kill_type = KILL;
	}

	seen_alnums = 0;
	
	while (tty->read_head != tty->canon_head) {
		head = tty->read_head;

		
		do {
			head = (head - 1) & (N_TTY_BUF_SIZE-1);
			c = tty->read_buf[head];
		} while (is_continuation(c, tty) && head != tty->canon_head);

		
		if (is_continuation(c, tty))
			break;

		if (kill_type == WERASE) {
			
			if (isalnum(c) || c == '_')
				seen_alnums++;
			else if (seen_alnums)
				break;
		}
		cnt = (tty->read_head - head) & (N_TTY_BUF_SIZE-1);
		spin_lock_irqsave(&tty->read_lock, flags);
		tty->read_head = head;
		tty->read_cnt -= cnt;
		spin_unlock_irqrestore(&tty->read_lock, flags);
		if (L_ECHO(tty)) {
			if (L_ECHOPRT(tty)) {
				if (!tty->erasing) {
					echo_char_raw('\\', tty);
					tty->erasing = 1;
				}
				
				echo_char(c, tty);
				while (--cnt > 0) {
					head = (head+1) & (N_TTY_BUF_SIZE-1);
					echo_char_raw(tty->read_buf[head], tty);
					echo_move_back_col(tty);
				}
			} else if (kill_type == ERASE && !L_ECHOE(tty)) {
				echo_char(ERASE_CHAR(tty), tty);
			} else if (c == '\t') {
				unsigned int num_chars = 0;
				int after_tab = 0;
				unsigned long tail = tty->read_head;

				while (tail != tty->canon_head) {
					tail = (tail-1) & (N_TTY_BUF_SIZE-1);
					c = tty->read_buf[tail];
					if (c == '\t') {
						after_tab = 1;
						break;
					} else if (iscntrl(c)) {
						if (L_ECHOCTL(tty))
							num_chars += 2;
					} else if (!is_continuation(c, tty)) {
						num_chars++;
					}
				}
				echo_erase_tab(num_chars, after_tab, tty);
			} else {
				if (iscntrl(c) && L_ECHOCTL(tty)) {
					echo_char_raw('\b', tty);
					echo_char_raw(' ', tty);
					echo_char_raw('\b', tty);
				}
				if (!iscntrl(c) || L_ECHOCTL(tty)) {
					echo_char_raw('\b', tty);
					echo_char_raw(' ', tty);
					echo_char_raw('\b', tty);
				}
			}
		}
		if (kill_type == ERASE)
			break;
	}
	if (tty->read_head == tty->canon_head && L_ECHO(tty))
		finish_erasing(tty);
}


static inline void isig(int sig, struct tty_struct *tty, int flush)
{
	if (tty->pgrp)
		kill_pgrp(tty->pgrp, sig, 1);
	if (flush || !L_NOFLSH(tty)) {
		n_tty_flush_buffer(tty);
		tty_driver_flush_buffer(tty);
	}
}


static inline void n_tty_receive_break(struct tty_struct *tty)
{
	if (I_IGNBRK(tty))
		return;
	if (I_BRKINT(tty)) {
		isig(SIGINT, tty, 1);
		return;
	}
	if (I_PARMRK(tty)) {
		put_tty_queue('\377', tty);
		put_tty_queue('\0', tty);
	}
	put_tty_queue('\0', tty);
	wake_up_interruptible(&tty->read_wait);
}


static inline void n_tty_receive_overrun(struct tty_struct *tty)
{
	char buf[64];

	tty->num_overrun++;
	if (time_before(tty->overrun_time, jiffies - HZ) ||
			time_after(tty->overrun_time, jiffies)) {
		printk(KERN_WARNING "%s: %d input overrun(s)\n",
			tty_name(tty, buf),
			tty->num_overrun);
		tty->overrun_time = jiffies;
		tty->num_overrun = 0;
	}
}

static inline void n_tty_receive_parity_error(struct tty_struct *tty,
					      unsigned char c)
{
	if (I_IGNPAR(tty))
		return;
	if (I_PARMRK(tty)) {
		put_tty_queue('\377', tty);
		put_tty_queue('\0', tty);
		put_tty_queue(c, tty);
	} else	if (I_INPCK(tty))
		put_tty_queue('\0', tty);
	else
		put_tty_queue(c, tty);
	wake_up_interruptible(&tty->read_wait);
}


static inline void n_tty_receive_char(struct tty_struct *tty, unsigned char c)
{
	unsigned long flags;
	int parmrk;

	if (tty->raw) {
		put_tty_queue(c, tty);
		return;
	}

	if (I_ISTRIP(tty))
		c &= 0x7f;
	if (I_IUCLC(tty) && L_IEXTEN(tty))
		c = tolower(c);

	if (L_EXTPROC(tty)) {
		put_tty_queue(c, tty);
		return;
	}

	if (tty->stopped && !tty->flow_stopped && I_IXON(tty) &&
	    I_IXANY(tty) && c != START_CHAR(tty) && c != STOP_CHAR(tty) &&
	    c != INTR_CHAR(tty) && c != QUIT_CHAR(tty) && c != SUSP_CHAR(tty)) {
		start_tty(tty);
		process_echoes(tty);
	}

	if (tty->closing) {
		if (I_IXON(tty)) {
			if (c == START_CHAR(tty)) {
				start_tty(tty);
				process_echoes(tty);
			} else if (c == STOP_CHAR(tty))
				stop_tty(tty);
		}
		return;
	}

	if (!test_bit(c, tty->process_char_map) || tty->lnext) {
		tty->lnext = 0;
		parmrk = (c == (unsigned char) '\377' && I_PARMRK(tty)) ? 1 : 0;
		if (tty->read_cnt >= (N_TTY_BUF_SIZE - parmrk - 1)) {
			
			if (L_ECHO(tty))
				process_output('\a', tty);
			return;
		}
		if (L_ECHO(tty)) {
			finish_erasing(tty);
			
			if (tty->canon_head == tty->read_head)
				echo_set_canon_col(tty);
			echo_char(c, tty);
			process_echoes(tty);
		}
		if (parmrk)
			put_tty_queue(c, tty);
		put_tty_queue(c, tty);
		return;
	}

	if (I_IXON(tty)) {
		if (c == START_CHAR(tty)) {
			start_tty(tty);
			process_echoes(tty);
			return;
		}
		if (c == STOP_CHAR(tty)) {
			stop_tty(tty);
			return;
		}
	}

	if (L_ISIG(tty)) {
		int signal;
		signal = SIGINT;
		if (c == INTR_CHAR(tty))
			goto send_signal;
		signal = SIGQUIT;
		if (c == QUIT_CHAR(tty))
			goto send_signal;
		signal = SIGTSTP;
		if (c == SUSP_CHAR(tty)) {
send_signal:
			if (!L_NOFLSH(tty)) {
				n_tty_flush_buffer(tty);
				tty_driver_flush_buffer(tty);
			}
			if (I_IXON(tty))
				start_tty(tty);
			if (L_ECHO(tty)) {
				echo_char(c, tty);
				process_echoes(tty);
			}
			if (tty->pgrp)
				kill_pgrp(tty->pgrp, signal, 1);
			return;
		}
	}

	if (c == '\r') {
		if (I_IGNCR(tty))
			return;
		if (I_ICRNL(tty))
			c = '\n';
	} else if (c == '\n' && I_INLCR(tty))
		c = '\r';

	if (tty->icanon) {
		if (c == ERASE_CHAR(tty) || c == KILL_CHAR(tty) ||
		    (c == WERASE_CHAR(tty) && L_IEXTEN(tty))) {
			eraser(c, tty);
			process_echoes(tty);
			return;
		}
		if (c == LNEXT_CHAR(tty) && L_IEXTEN(tty)) {
			tty->lnext = 1;
			if (L_ECHO(tty)) {
				finish_erasing(tty);
				if (L_ECHOCTL(tty)) {
					echo_char_raw('^', tty);
					echo_char_raw('\b', tty);
					process_echoes(tty);
				}
			}
			return;
		}
		if (c == REPRINT_CHAR(tty) && L_ECHO(tty) &&
		    L_IEXTEN(tty)) {
			unsigned long tail = tty->canon_head;

			finish_erasing(tty);
			echo_char(c, tty);
			echo_char_raw('\n', tty);
			while (tail != tty->read_head) {
				echo_char(tty->read_buf[tail], tty);
				tail = (tail+1) & (N_TTY_BUF_SIZE-1);
			}
			process_echoes(tty);
			return;
		}
		if (c == '\n') {
			if (tty->read_cnt >= N_TTY_BUF_SIZE) {
				if (L_ECHO(tty))
					process_output('\a', tty);
				return;
			}
			if (L_ECHO(tty) || L_ECHONL(tty)) {
				echo_char_raw('\n', tty);
				process_echoes(tty);
			}
			goto handle_newline;
		}
		if (c == EOF_CHAR(tty)) {
			if (tty->read_cnt >= N_TTY_BUF_SIZE)
				return;
			if (tty->canon_head != tty->read_head)
				set_bit(TTY_PUSH, &tty->flags);
			c = __DISABLED_CHAR;
			goto handle_newline;
		}
		if ((c == EOL_CHAR(tty)) ||
		    (c == EOL2_CHAR(tty) && L_IEXTEN(tty))) {
			parmrk = (c == (unsigned char) '\377' && I_PARMRK(tty))
				 ? 1 : 0;
			if (tty->read_cnt >= (N_TTY_BUF_SIZE - parmrk)) {
				if (L_ECHO(tty))
					process_output('\a', tty);
				return;
			}
			if (L_ECHO(tty)) {
				
				if (tty->canon_head == tty->read_head)
					echo_set_canon_col(tty);
				echo_char(c, tty);
				process_echoes(tty);
			}
			if (parmrk)
				put_tty_queue(c, tty);

handle_newline:
			spin_lock_irqsave(&tty->read_lock, flags);
			set_bit(tty->read_head, tty->read_flags);
			put_tty_queue_nolock(c, tty);
			tty->canon_head = tty->read_head;
			tty->canon_data++;
			spin_unlock_irqrestore(&tty->read_lock, flags);
			kill_fasync(&tty->fasync, SIGIO, POLL_IN);
			if (waitqueue_active(&tty->read_wait))
				wake_up_interruptible(&tty->read_wait);
			return;
		}
	}

	parmrk = (c == (unsigned char) '\377' && I_PARMRK(tty)) ? 1 : 0;
	if (tty->read_cnt >= (N_TTY_BUF_SIZE - parmrk - 1)) {
		
		if (L_ECHO(tty))
			process_output('\a', tty);
		return;
	}
	if (L_ECHO(tty)) {
		finish_erasing(tty);
		if (c == '\n')
			echo_char_raw('\n', tty);
		else {
			
			if (tty->canon_head == tty->read_head)
				echo_set_canon_col(tty);
			echo_char(c, tty);
		}
		process_echoes(tty);
	}

	if (parmrk)
		put_tty_queue(c, tty);

	put_tty_queue(c, tty);
}



static void n_tty_write_wakeup(struct tty_struct *tty)
{
	if (tty->fasync && test_and_clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags))
		kill_fasync(&tty->fasync, SIGIO, POLL_OUT);
}


static void n_tty_receive_buf(struct tty_struct *tty, const unsigned char *cp,
			      char *fp, int count)
{
	const unsigned char *p;
	char *f, flags = TTY_NORMAL;
	int	i;
	char	buf[64];
	unsigned long cpuflags;

	if (!tty->read_buf)
		return;

	if (tty->real_raw) {
		spin_lock_irqsave(&tty->read_lock, cpuflags);
		i = min(N_TTY_BUF_SIZE - tty->read_cnt,
			N_TTY_BUF_SIZE - tty->read_head);
		i = min(count, i);
		memcpy(tty->read_buf + tty->read_head, cp, i);
		tty->read_head = (tty->read_head + i) & (N_TTY_BUF_SIZE-1);
		tty->read_cnt += i;
		cp += i;
		count -= i;

		i = min(N_TTY_BUF_SIZE - tty->read_cnt,
			N_TTY_BUF_SIZE - tty->read_head);
		i = min(count, i);
		memcpy(tty->read_buf + tty->read_head, cp, i);
		tty->read_head = (tty->read_head + i) & (N_TTY_BUF_SIZE-1);
		tty->read_cnt += i;
		spin_unlock_irqrestore(&tty->read_lock, cpuflags);
	} else {
		for (i = count, p = cp, f = fp; i; i--, p++) {
			if (f)
				flags = *f++;
			switch (flags) {
			case TTY_NORMAL:
				n_tty_receive_char(tty, *p);
				break;
			case TTY_BREAK:
				n_tty_receive_break(tty);
				break;
			case TTY_PARITY:
			case TTY_FRAME:
				n_tty_receive_parity_error(tty, *p);
				break;
			case TTY_OVERRUN:
				n_tty_receive_overrun(tty);
				break;
			default:
				printk(KERN_ERR "%s: unknown flag %d\n",
				       tty_name(tty, buf), flags);
				break;
			}
		}
		if (tty->ops->flush_chars)
			tty->ops->flush_chars(tty);
	}

	n_tty_set_room(tty);

	if ((!tty->icanon && (tty->read_cnt >= tty->minimum_to_wake)) ||
		L_EXTPROC(tty)) {
		kill_fasync(&tty->fasync, SIGIO, POLL_IN);
		if (waitqueue_active(&tty->read_wait))
			wake_up_interruptible(&tty->read_wait);
	}

	if (tty->receive_room < TTY_THRESHOLD_THROTTLE)
		tty_throttle(tty);
}

int is_ignored(int sig)
{
	return (sigismember(&current->blocked, sig) ||
		current->sighand->action[sig-1].sa.sa_handler == SIG_IGN);
}


static void n_tty_set_termios(struct tty_struct *tty, struct ktermios *old)
{
	int canon_change = 1;
	BUG_ON(!tty);

	if (old)
		canon_change = (old->c_lflag ^ tty->termios->c_lflag) & ICANON;
	if (canon_change) {
		memset(&tty->read_flags, 0, sizeof tty->read_flags);
		tty->canon_head = tty->read_tail;
		tty->canon_data = 0;
		tty->erasing = 0;
	}

	if (canon_change && !L_ICANON(tty) && tty->read_cnt)
		wake_up_interruptible(&tty->read_wait);

	tty->icanon = (L_ICANON(tty) != 0);
	if (test_bit(TTY_HW_COOK_IN, &tty->flags)) {
		tty->raw = 1;
		tty->real_raw = 1;
		n_tty_set_room(tty);
		return;
	}
	if (I_ISTRIP(tty) || I_IUCLC(tty) || I_IGNCR(tty) ||
	    I_ICRNL(tty) || I_INLCR(tty) || L_ICANON(tty) ||
	    I_IXON(tty) || L_ISIG(tty) || L_ECHO(tty) ||
	    I_PARMRK(tty)) {
		memset(tty->process_char_map, 0, 256/8);

		if (I_IGNCR(tty) || I_ICRNL(tty))
			set_bit('\r', tty->process_char_map);
		if (I_INLCR(tty))
			set_bit('\n', tty->process_char_map);

		if (L_ICANON(tty)) {
			set_bit(ERASE_CHAR(tty), tty->process_char_map);
			set_bit(KILL_CHAR(tty), tty->process_char_map);
			set_bit(EOF_CHAR(tty), tty->process_char_map);
			set_bit('\n', tty->process_char_map);
			set_bit(EOL_CHAR(tty), tty->process_char_map);
			if (L_IEXTEN(tty)) {
				set_bit(WERASE_CHAR(tty),
					tty->process_char_map);
				set_bit(LNEXT_CHAR(tty),
					tty->process_char_map);
				set_bit(EOL2_CHAR(tty),
					tty->process_char_map);
				if (L_ECHO(tty))
					set_bit(REPRINT_CHAR(tty),
						tty->process_char_map);
			}
		}
		if (I_IXON(tty)) {
			set_bit(START_CHAR(tty), tty->process_char_map);
			set_bit(STOP_CHAR(tty), tty->process_char_map);
		}
		if (L_ISIG(tty)) {
			set_bit(INTR_CHAR(tty), tty->process_char_map);
			set_bit(QUIT_CHAR(tty), tty->process_char_map);
			set_bit(SUSP_CHAR(tty), tty->process_char_map);
		}
		clear_bit(__DISABLED_CHAR, tty->process_char_map);
		tty->raw = 0;
		tty->real_raw = 0;
	} else {
		tty->raw = 1;
		if ((I_IGNBRK(tty) || (!I_BRKINT(tty) && !I_PARMRK(tty))) &&
		    (I_IGNPAR(tty) || !I_INPCK(tty)) &&
		    (tty->driver->flags & TTY_DRIVER_REAL_RAW))
			tty->real_raw = 1;
		else
			tty->real_raw = 0;
	}
	n_tty_set_room(tty);
	
	wake_up_interruptible(&tty->write_wait);
	wake_up_interruptible(&tty->read_wait);
}


static void n_tty_close(struct tty_struct *tty)
{
	n_tty_flush_buffer(tty);
	if (tty->read_buf) {
		kfree(tty->read_buf);
		tty->read_buf = NULL;
	}
	if (tty->echo_buf) {
		kfree(tty->echo_buf);
		tty->echo_buf = NULL;
	}
}


static int n_tty_open(struct tty_struct *tty)
{
	if (!tty)
		return -EINVAL;

	
	if (!tty->read_buf) {
		tty->read_buf = kzalloc(N_TTY_BUF_SIZE, GFP_KERNEL);
		if (!tty->read_buf)
			return -ENOMEM;
	}
	if (!tty->echo_buf) {
		tty->echo_buf = kzalloc(N_TTY_BUF_SIZE, GFP_KERNEL);

		if (!tty->echo_buf)
			return -ENOMEM;
	}
	reset_buffer_flags(tty);
	tty_unthrottle(tty);
	tty->column = 0;
	n_tty_set_termios(tty, NULL);
	tty->minimum_to_wake = 1;
	tty->closing = 0;
	return 0;
}

static inline int input_available_p(struct tty_struct *tty, int amt)
{
	tty_flush_to_ldisc(tty);
	if (tty->icanon && !L_EXTPROC(tty)) {
		if (tty->canon_data)
			return 1;
	} else if (tty->read_cnt >= (amt ? amt : 1))
		return 1;

	return 0;
}


static int copy_from_read_buf(struct tty_struct *tty,
				      unsigned char __user **b,
				      size_t *nr)

{
	int retval;
	size_t n;
	unsigned long flags;

	retval = 0;
	spin_lock_irqsave(&tty->read_lock, flags);
	n = min(tty->read_cnt, N_TTY_BUF_SIZE - tty->read_tail);
	n = min(*nr, n);
	spin_unlock_irqrestore(&tty->read_lock, flags);
	if (n) {
		retval = copy_to_user(*b, &tty->read_buf[tty->read_tail], n);
		n -= retval;
		tty_audit_add_data(tty, &tty->read_buf[tty->read_tail], n);
		spin_lock_irqsave(&tty->read_lock, flags);
		tty->read_tail = (tty->read_tail + n) & (N_TTY_BUF_SIZE-1);
		tty->read_cnt -= n;
		
		if (L_EXTPROC(tty) && tty->icanon && n == 1) {
			if (!tty->read_cnt && (*b)[n-1] == EOF_CHAR(tty))
				n--;
		}
		spin_unlock_irqrestore(&tty->read_lock, flags);
		*b += n;
		*nr -= n;
	}
	return retval;
}

extern ssize_t redirected_tty_write(struct file *, const char __user *,
							size_t, loff_t *);


static int job_control(struct tty_struct *tty, struct file *file)
{
	
	if (file->f_op->write != redirected_tty_write &&
	    current->signal->tty == tty) {
		if (!tty->pgrp)
			printk(KERN_ERR "n_tty_read: no tty->pgrp!\n");
		else if (task_pgrp(current) != tty->pgrp) {
			if (is_ignored(SIGTTIN) ||
			    is_current_pgrp_orphaned())
				return -EIO;
			kill_pgrp(task_pgrp(current), SIGTTIN, 1);
			set_thread_flag(TIF_SIGPENDING);
			return -ERESTARTSYS;
		}
	}
	return 0;
}



static ssize_t n_tty_read(struct tty_struct *tty, struct file *file,
			 unsigned char __user *buf, size_t nr)
{
	unsigned char __user *b = buf;
	DECLARE_WAITQUEUE(wait, current);
	int c;
	int minimum, time;
	ssize_t retval = 0;
	ssize_t size;
	long timeout;
	unsigned long flags;
	int packet;

do_it_again:

	BUG_ON(!tty->read_buf);

	c = job_control(tty, file);
	if (c < 0)
		return c;

	minimum = time = 0;
	timeout = MAX_SCHEDULE_TIMEOUT;
	if (!tty->icanon) {
		time = (HZ / 10) * TIME_CHAR(tty);
		minimum = MIN_CHAR(tty);
		if (minimum) {
			if (time)
				tty->minimum_to_wake = 1;
			else if (!waitqueue_active(&tty->read_wait) ||
				 (tty->minimum_to_wake > minimum))
				tty->minimum_to_wake = minimum;
		} else {
			timeout = 0;
			if (time) {
				timeout = time;
				time = 0;
			}
			tty->minimum_to_wake = minimum = 1;
		}
	}

	if (file->f_flags & O_NONBLOCK) {
		if (!mutex_trylock(&tty->atomic_read_lock))
			return -EAGAIN;
	} else {
		if (mutex_lock_interruptible(&tty->atomic_read_lock))
			return -ERESTARTSYS;
	}
	packet = tty->packet;

	add_wait_queue(&tty->read_wait, &wait);
	while (nr) {
		
		if (packet && tty->link->ctrl_status) {
			unsigned char cs;
			if (b != buf)
				break;
			spin_lock_irqsave(&tty->link->ctrl_lock, flags);
			cs = tty->link->ctrl_status;
			tty->link->ctrl_status = 0;
			spin_unlock_irqrestore(&tty->link->ctrl_lock, flags);
			if (tty_put_user(tty, cs, b++)) {
				retval = -EFAULT;
				b--;
				break;
			}
			nr--;
			break;
		}
		set_current_state(TASK_INTERRUPTIBLE);

		if (((minimum - (b - buf)) < tty->minimum_to_wake) &&
		    ((minimum - (b - buf)) >= 1))
			tty->minimum_to_wake = (minimum - (b - buf));

		if (!input_available_p(tty, 0)) {
			if (test_bit(TTY_OTHER_CLOSED, &tty->flags)) {
				retval = -EIO;
				break;
			}
			if (tty_hung_up_p(file))
				break;
			if (!timeout)
				break;
			if (file->f_flags & O_NONBLOCK) {
				retval = -EAGAIN;
				break;
			}
			if (signal_pending(current)) {
				retval = -ERESTARTSYS;
				break;
			}
			n_tty_set_room(tty);
			timeout = schedule_timeout(timeout);
			BUG_ON(!tty->read_buf);
			continue;
		}
		__set_current_state(TASK_RUNNING);

		
		if (packet && b == buf) {
			if (tty_put_user(tty, TIOCPKT_DATA, b++)) {
				retval = -EFAULT;
				b--;
				break;
			}
			nr--;
		}

		if (tty->icanon && !L_EXTPROC(tty)) {
			
			while (nr && tty->read_cnt) {
				int eol;

				eol = test_and_clear_bit(tty->read_tail,
						tty->read_flags);
				c = tty->read_buf[tty->read_tail];
				spin_lock_irqsave(&tty->read_lock, flags);
				tty->read_tail = ((tty->read_tail+1) &
						  (N_TTY_BUF_SIZE-1));
				tty->read_cnt--;
				if (eol) {
					if (--tty->canon_data < 0)
						tty->canon_data = 0;
				}
				spin_unlock_irqrestore(&tty->read_lock, flags);

				if (!eol || (c != __DISABLED_CHAR)) {
					if (tty_put_user(tty, c, b++)) {
						retval = -EFAULT;
						b--;
						break;
					}
					nr--;
				}
				if (eol) {
					tty_audit_push(tty);
					break;
				}
			}
			if (retval)
				break;
		} else {
			int uncopied;
			uncopied = copy_from_read_buf(tty, &b, &nr);
			uncopied += copy_from_read_buf(tty, &b, &nr);
			if (uncopied) {
				retval = -EFAULT;
				break;
			}
		}

		if (n_tty_chars_in_buffer(tty) <= TTY_THRESHOLD_UNTHROTTLE) {
			n_tty_set_room(tty);
			check_unthrottle(tty);
		}

		if (b - buf >= minimum)
			break;
		if (time)
			timeout = time;
	}
	mutex_unlock(&tty->atomic_read_lock);
	remove_wait_queue(&tty->read_wait, &wait);

	if (!waitqueue_active(&tty->read_wait))
		tty->minimum_to_wake = minimum;

	__set_current_state(TASK_RUNNING);
	size = b - buf;
	if (size) {
		retval = size;
		if (nr)
			clear_bit(TTY_PUSH, &tty->flags);
	} else if (test_and_clear_bit(TTY_PUSH, &tty->flags))
		goto do_it_again;

	n_tty_set_room(tty);
	return retval;
}


static ssize_t n_tty_write(struct tty_struct *tty, struct file *file,
			   const unsigned char *buf, size_t nr)
{
	const unsigned char *b = buf;
	DECLARE_WAITQUEUE(wait, current);
	int c;
	ssize_t retval = 0;

	
	if (L_TOSTOP(tty) && file->f_op->write != redirected_tty_write) {
		retval = tty_check_change(tty);
		if (retval)
			return retval;
	}

	
	process_echoes(tty);

	add_wait_queue(&tty->write_wait, &wait);
	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (signal_pending(current)) {
			retval = -ERESTARTSYS;
			break;
		}
		if (tty_hung_up_p(file) || (tty->link && !tty->link->count)) {
			retval = -EIO;
			break;
		}
		if (O_OPOST(tty) && !(test_bit(TTY_HW_COOK_OUT, &tty->flags))) {
			while (nr > 0) {
				ssize_t num = process_output_block(tty, b, nr);
				if (num < 0) {
					if (num == -EAGAIN)
						break;
					retval = num;
					goto break_out;
				}
				b += num;
				nr -= num;
				if (nr == 0)
					break;
				c = *b;
				if (process_output(c, tty) < 0)
					break;
				b++; nr--;
			}
			if (tty->ops->flush_chars)
				tty->ops->flush_chars(tty);
		} else {

			while (nr > 0) {
				mutex_lock(&tty->output_lock);
				c = tty->ops->write(tty, b, nr);
				mutex_unlock(&tty->output_lock);
				if (c < 0) {
					retval = c;
					goto break_out;
				}
				if (!c)
					break;
				b += c;
				nr -= c;
			}
		}
		if (!nr)
			break;
		if (file->f_flags & O_NONBLOCK) {
			retval = -EAGAIN;
			break;
		}
		schedule();
	}
break_out:
	__set_current_state(TASK_RUNNING);
	remove_wait_queue(&tty->write_wait, &wait);
	if (b - buf != nr && tty->fasync)
		set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	return (b - buf) ? b - buf : retval;
}


static unsigned int n_tty_poll(struct tty_struct *tty, struct file *file,
							poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &tty->read_wait, wait);
	poll_wait(file, &tty->write_wait, wait);
	if (input_available_p(tty, TIME_CHAR(tty) ? 0 : MIN_CHAR(tty)))
		mask |= POLLIN | POLLRDNORM;
	if (tty->packet && tty->link->ctrl_status)
		mask |= POLLPRI | POLLIN | POLLRDNORM;
	if (test_bit(TTY_OTHER_CLOSED, &tty->flags))
		mask |= POLLHUP;
	if (tty_hung_up_p(file))
		mask |= POLLHUP;
	if (!(mask & (POLLHUP | POLLIN | POLLRDNORM))) {
		if (MIN_CHAR(tty) && !TIME_CHAR(tty))
			tty->minimum_to_wake = MIN_CHAR(tty);
		else
			tty->minimum_to_wake = 1;
	}
	if (tty->ops->write && !tty_is_writelocked(tty) &&
			tty_chars_in_buffer(tty) < WAKEUP_CHARS &&
			tty_write_room(tty) > 0)
		mask |= POLLOUT | POLLWRNORM;
	return mask;
}

static unsigned long inq_canon(struct tty_struct *tty)
{
	int nr, head, tail;

	if (!tty->canon_data)
		return 0;
	head = tty->canon_head;
	tail = tty->read_tail;
	nr = (head - tail) & (N_TTY_BUF_SIZE-1);
	
	while (head != tail) {
		if (test_bit(tail, tty->read_flags) &&
		    tty->read_buf[tail] == __DISABLED_CHAR)
			nr--;
		tail = (tail+1) & (N_TTY_BUF_SIZE-1);
	}
	return nr;
}

static int n_tty_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	int retval;

	switch (cmd) {
	case TIOCOUTQ:
		return put_user(tty_chars_in_buffer(tty), (int __user *) arg);
	case TIOCINQ:
		
		retval = tty->read_cnt;
		if (L_ICANON(tty))
			retval = inq_canon(tty);
		return put_user(retval, (unsigned int __user *) arg);
	default:
		return n_tty_ioctl_helper(tty, file, cmd, arg);
	}
}

struct tty_ldisc_ops tty_ldisc_N_TTY = {
	.magic           = TTY_LDISC_MAGIC,
	.name            = "n_tty",
	.open            = n_tty_open,
	.close           = n_tty_close,
	.flush_buffer    = n_tty_flush_buffer,
	.chars_in_buffer = n_tty_chars_in_buffer,
	.read            = n_tty_read,
	.write           = n_tty_write,
	.ioctl           = n_tty_ioctl,
	.set_termios     = n_tty_set_termios,
	.poll            = n_tty_poll,
	.receive_buf     = n_tty_receive_buf,
	.write_wakeup    = n_tty_write_wakeup
};


void n_tty_inherit_ops(struct tty_ldisc_ops *ops)
{
	*ops = tty_ldisc_N_TTY;
	ops->owner = NULL;
	ops->refcount = ops->flags = 0;
}
EXPORT_SYMBOL_GPL(n_tty_inherit_ops);
