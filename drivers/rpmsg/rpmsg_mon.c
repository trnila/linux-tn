#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/sched/signal.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/export.h>
#include <linux/usb.h>
#include <linux/poll.h>
#include <linux/compat.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/time64.h>

#include <linux/uaccess.h>

/*
 *  * Defined by USB 2.0 clause 9.3, table 9.2.
 *   */
#define SETUP_LEN  8

/* ioctl macros */
#define MON_IOC_MAGIC 0x92

#define MON_IOCQ_URB_LEN _IO(MON_IOC_MAGIC, 1)
/* #2 used to be MON_IOCX_URB, removed before it got into Linus tree */
#define MON_IOCG_STATS _IOR(MON_IOC_MAGIC, 3, struct mon_bin_stats)
#define MON_IOCT_RING_SIZE _IO(MON_IOC_MAGIC, 4)
#define MON_IOCQ_RING_SIZE _IO(MON_IOC_MAGIC, 5)
#define MON_IOCX_GET   _IOW(MON_IOC_MAGIC, 6, struct mon_bin_get)
#define MON_IOCX_MFETCH _IOWR(MON_IOC_MAGIC, 7, struct mon_bin_mfetch)
#define MON_IOCH_MFLUSH _IO(MON_IOC_MAGIC, 8)
/* #9 was MON_IOCT_SETAPI */
#define MON_IOCX_GETX   _IOW(MON_IOC_MAGIC, 10, struct mon_bin_get)

#ifdef CONFIG_COMPAT
#define MON_IOCX_GET32 _IOW(MON_IOC_MAGIC, 6, struct mon_bin_get32)
#define MON_IOCX_MFETCH32 _IOWR(MON_IOC_MAGIC, 7, struct mon_bin_mfetch32)
#define MON_IOCX_GETX32   _IOW(MON_IOC_MAGIC, 10, struct mon_bin_get32)
#endif

/*
 *  * Some architectures have enormous basic pages (16KB for ia64, 64KB for ppc).
 *   * But it's all right. Just use a simple way to make sure the chunk is never
 *    * smaller than a page.
 *     *
 *      * N.B. An application does not know our chunk size.
 *       *
 *        * Woops, get_zeroed_page() returns a single page. I guess we're stuck with
 *         * page-sized chunks for the time being.
 *          */
#define CHUNK_SIZE   PAGE_SIZE
#define CHUNK_ALIGN(x)   (((x)+CHUNK_SIZE-1) & ~(CHUNK_SIZE-1))

/*
 *  * The magic limit was calculated so that it allows the monitoring
 *   * application to pick data once in two ticks. This way, another application,
 *    * which presumably drives the bus, gets to hog CPU, yet we collect our data.
 *     * If HZ is 100, a 480 mbit/s bus drives 614 KB every jiffy. USB has an
 *      * enormous overhead built into the bus protocol, so we need about 1000 KB.
 *       *
 *        * This is still too much for most cases, where we just snoop a few
 *         * descriptor fetches for enumeration. So, the default is a "reasonable"
 *          * amount for systems with HZ=250 and incomplete bus saturation.
 *           *
 *            * XXX What about multi-megabyte URBs which take minutes to transfer?
 *             */
#define BUFF_MAX  CHUNK_ALIGN(1200*1024)
#define BUFF_DFL   CHUNK_ALIGN(300*1024)
#define BUFF_MIN     CHUNK_ALIGN(8*1024)

#define MON_BIN_MAX_MINOR 128
static struct class *mon_bin_class;
static dev_t mon_bin_dev0;
static struct cdev mon_bin_cdev;
static struct device *dev;
static DEFINE_MUTEX(mon_lock);

struct rpmsg_hdr {
	u32 src;
	u32 dst;
	u32 reserved;
	u16 len;
	u16 flags;
	u8 data[0];
};

struct mon_bin_hdr {
	u64 timestamp;
	u32 interface;
	u16 vq;
	u16 res;
	struct rpmsg_hdr hdr;
};

/* per file statistic */
struct mon_bin_stats {
	u32 queued;
	u32 dropped;
};

struct mon_bin_get {
	struct mon_bin_hdr __user *hdr;	/* Can be 48 bytes or 64. */
	void __user *data;
	size_t alloc;		/* Length of data (can be zero) */
};

struct mon_bin_mfetch {
	u32 __user *offvec;	/* Vector of events fetched */
	u32 nfetch;		/* Number of events to fetch (out: fetched) */
	u32 nflush;		/* Number of events to flush */
};

#ifdef CONFIG_COMPAT
struct mon_bin_get32 {
	u32 hdr32;
	u32 data32;
	u32 alloc32;
};

struct mon_bin_mfetch32 {
        u32 offvec32;
        u32 nfetch32;
        u32 nflush32;
};
#endif
struct mon_reader_bin *rp;

/* Having these two values same prevents wrapping of the mon_bin_hdr */
#define PKT_ALIGN   32
#define PKT_SIZE    32

/* max number of USB bus supported */
#define MON_BIN_MAX_MINOR 128

/*
 * The buffer: map of used pages.
 */
struct mon_pgmap {
	struct page *pg;
	unsigned char *ptr;	/* XXX just use page_to_virt everywhere? */
};

/*
 * This gets associated with an open file struct.
 */
struct mon_reader_bin {
	/* The buffer: one per open. */
	spinlock_t b_lock;		/* Protect b_cnt, b_in */
	unsigned int b_size;		/* Current size of the buffer - bytes */
	unsigned int b_cnt;		/* Bytes used */
	unsigned int b_in, b_out;	/* Offsets into buffer - bytes */
	unsigned int b_read;		/* Amount of read data in curr. pkt. */
	struct mon_pgmap *b_vec;	/* The map array */
	wait_queue_head_t b_wait;	/* Wait for data here */

	struct mutex fetch_lock;	/* Protect b_read, b_out */
	int mmap_active;

	/* A list of these is needed for "bus 0". Some time later. */
//	struct mon_reader r;

	/* Stats */
	unsigned int cnt_lost;
};

static inline struct mon_bin_hdr *MON_OFF2HDR(const struct mon_reader_bin *rp,
    unsigned int offset)
{
	return (struct mon_bin_hdr *)
	    (rp->b_vec[offset / CHUNK_SIZE].ptr + offset % CHUNK_SIZE);
}

#define MON_RING_EMPTY(rp)	((rp)->b_cnt == 0)

static struct class *mon_bin_class;
static dev_t mon_bin_dev0;
static struct cdev mon_bin_cdev;

static void mon_buff_area_fill(const struct mon_reader_bin *rp,
    unsigned int offset, unsigned int size);
static int mon_bin_wait_event(struct file *file, struct mon_reader_bin *rp);
static int mon_alloc_buff(struct mon_pgmap *map, int npages);
static void mon_free_buff(struct mon_pgmap *map, int npages);

/*
 * This is a "chunked memcpy". It does not manipulate any counters.
 */
static unsigned int mon_copy_to_buff(const struct mon_reader_bin *this,
    unsigned int off, const unsigned char *from, unsigned int length)
{
	unsigned int step_len;
	unsigned char *buf;
	unsigned int in_page;

	while (length) {
		/*
		 * Determine step_len.
		 */
		step_len = length;
		in_page = CHUNK_SIZE - (off & (CHUNK_SIZE-1));
		if (in_page < step_len)
			step_len = in_page;

		/*
		 * Copy data and advance pointers.
		 */
		buf = this->b_vec[off / CHUNK_SIZE].ptr + off % CHUNK_SIZE;
		memcpy(buf, from, step_len);
		if ((off += step_len) >= this->b_size) off = 0;
		from += step_len;
		length -= step_len;
	}
	return off;
}

/*
 * This is a little worse than the above because it's "chunked copy_to_user".
 * The return value is an error code, not an offset.
 */
static int copy_from_buf(const struct mon_reader_bin *this, unsigned int off,
    char __user *to, int length)
{
	unsigned int step_len;
	unsigned char *buf;
	unsigned int in_page;

	while (length) {
		/*
		 * Determine step_len.
		 */
		step_len = length;
		in_page = CHUNK_SIZE - (off & (CHUNK_SIZE-1));
		if (in_page < step_len)
			step_len = in_page;

		/*
		 * Copy data and advance pointers.
		 */
		buf = this->b_vec[off / CHUNK_SIZE].ptr + off % CHUNK_SIZE;
		if (copy_to_user(to, buf, step_len))
			return -EINVAL;
		if ((off += step_len) >= this->b_size) off = 0;
		to += step_len;
		length -= step_len;
	}
	return 0;
}

/*
 * Allocate an (aligned) area in the buffer.
 * This is called under b_lock.
 * Returns ~0 on failure.
 */
static unsigned int mon_buff_area_alloc(struct mon_reader_bin *rp,
    unsigned int size)
{
	unsigned int offset;

	size = (size + PKT_ALIGN-1) & ~(PKT_ALIGN-1);
	if (rp->b_cnt + size > rp->b_size)
		return ~0;
	offset = rp->b_in;
	rp->b_cnt += size;
	if ((rp->b_in += size) >= rp->b_size)
		rp->b_in -= rp->b_size;
	return offset;
}

/*
 * This is the same thing as mon_buff_area_alloc, only it does not allow
 * buffers to wrap. This is needed by applications which pass references
 * into mmap-ed buffers up their stacks (libpcap can do that).
 *
 * Currently, we always have the header stuck with the data, although
 * it is not strictly speaking necessary.
 *
 * When a buffer would wrap, we place a filler packet to mark the space.
 */
static unsigned int mon_buff_area_alloc_contiguous(struct mon_reader_bin *rp,
    unsigned int size)
{
	unsigned int offset;
	unsigned int fill_size;

	size = (size + PKT_ALIGN-1) & ~(PKT_ALIGN-1);
	if (rp->b_cnt + size > rp->b_size)
		return ~0;
	if (rp->b_in + size > rp->b_size) {
		/*
		 * This would wrap. Find if we still have space after
		 * skipping to the end of the buffer. If we do, place
		 * a filler packet and allocate a new packet.
		 */
		fill_size = rp->b_size - rp->b_in;
		if (rp->b_cnt + size + fill_size > rp->b_size)
			return ~0;
		mon_buff_area_fill(rp, rp->b_in, fill_size);

		offset = 0;
		rp->b_in = size;
		rp->b_cnt += size + fill_size;
	} else if (rp->b_in + size == rp->b_size) {
		offset = rp->b_in;
		rp->b_in = 0;
		rp->b_cnt += size;
	} else {
		offset = rp->b_in;
		rp->b_in += size;
		rp->b_cnt += size;
	}
	return offset;
}

/*
 * Return a few (kilo-)bytes to the head of the buffer.
 * This is used if a data fetch fails.
 */
static void mon_buff_area_shrink(struct mon_reader_bin *rp, unsigned int size)
{

	/* size &= ~(PKT_ALIGN-1);  -- we're called with aligned size */
	rp->b_cnt -= size;
	if (rp->b_in < size)
		rp->b_in += rp->b_size;
	rp->b_in -= size;
}

/*
 * This has to be called under both b_lock and fetch_lock, because
 * it accesses both b_cnt and b_out.
 */
static void mon_buff_area_free(struct mon_reader_bin *rp, unsigned int size)
{

	size = (size + PKT_ALIGN-1) & ~(PKT_ALIGN-1);
	rp->b_cnt -= size;
	if ((rp->b_out += size) >= rp->b_size)
		rp->b_out -= rp->b_size;
}

static void mon_buff_area_fill(const struct mon_reader_bin *rp,
    unsigned int offset, unsigned int size)
{
	struct mon_bin_hdr *ep;

	ep = MON_OFF2HDR(rp, offset);
	memset(ep, 0, PKT_SIZE);
	ep->hdr.len = size - PKT_SIZE;
}

static inline char mon_bin_get_setup(unsigned char *setupb,
    const struct urb *urb, char ev_type)
{

	if (urb->setup_packet == NULL)
		return 'Z';
	memcpy(setupb, urb->setup_packet, SETUP_LEN);
	return 0;
}

static void cb(void *data, unsigned int l) {
	struct timespec64 ts;
	unsigned long flags;
	unsigned int offset;
	unsigned int delta;
	struct mon_bin_hdr *ep;
	struct rpmsg_hdr *hdr = data;
	unsigned int length = hdr->len;

	if(!rp) return;

	ktime_get_real_ts64(&ts);
	spin_lock_irqsave(&rp->b_lock, flags);

	if (length >= rp->b_size/5)
		length = rp->b_size/5;

	if (rp->mmap_active) {
		offset = mon_buff_area_alloc_contiguous(rp,
						 length + PKT_SIZE);
	} else {
		offset = mon_buff_area_alloc(rp, length + PKT_SIZE);
	}
	if (offset == ~0) {
		rp->cnt_lost++;
		spin_unlock_irqrestore(&rp->b_lock, flags);
		return;
	}

	ep = MON_OFF2HDR(rp, offset);
	if ((offset += PKT_SIZE) >= rp->b_size) offset = 0;
	ep->timestamp = ktime_get();
	ep->interface = 0;
	ep->vq = 0;
	memcpy(&ep->hdr, data, sizeof(ep->hdr));

	printk("to_buf: %d %d\n", offset, length);
	length = mon_copy_to_buff(rp, offset, data + sizeof(ep->hdr), length);
	printk("to_buf: %d %d\n", offset, length);
	if (length > 0) {
		delta = (ep->hdr.len + PKT_ALIGN-1) & ~(PKT_ALIGN-1);
//		ep->hdr.len -= length;
		delta -= (ep->hdr.len + PKT_ALIGN-1) & ~(PKT_ALIGN-1);
		mon_buff_area_shrink(rp, delta);
	}

	spin_unlock_irqrestore(&rp->b_lock, flags);

	wake_up(&rp->b_wait);
}

static int mon_bin_open(struct inode *inode, struct file *file)
{
	size_t size;
	int rc;

	rp = kzalloc(sizeof(struct mon_reader_bin), GFP_KERNEL);
	if (rp == NULL) {
		rc = -ENOMEM;
		goto err_alloc;
	}
	spin_lock_init(&rp->b_lock);
	init_waitqueue_head(&rp->b_wait);
	mutex_init(&rp->fetch_lock);
	rp->b_size = BUFF_DFL;

	size = sizeof(struct mon_pgmap) * (rp->b_size/CHUNK_SIZE);
	if ((rp->b_vec = kzalloc(size, GFP_KERNEL)) == NULL) {
		rc = -ENOMEM;
		goto err_allocvec;
	}

	if ((rc = mon_alloc_buff(rp->b_vec, rp->b_size/CHUNK_SIZE)) < 0)
		goto err_allocbuff;

	file->private_data = rp;
	mutex_unlock(&mon_lock);
	return 0;

err_allocbuff:
	kfree(rp->b_vec);
err_allocvec:
	kfree(rp);
err_alloc:
	mutex_unlock(&mon_lock);
	return rc;
}

/*
 * Extract an event from buffer and copy it to user space.
 * Wait if there is no event ready.
 * Returns zero or error.
 */
static int mon_bin_get_event(struct file *file, struct mon_reader_bin *rp,
    struct mon_bin_hdr __user *hdr, unsigned int hdrbytes,
    void __user *data, unsigned int nbytes)
{
	unsigned long flags;
	struct mon_bin_hdr *ep;
	size_t step_len;
	unsigned int offset;
	int rc;

	mutex_lock(&rp->fetch_lock);

	if ((rc = mon_bin_wait_event(file, rp)) < 0) {
		mutex_unlock(&rp->fetch_lock);
		return rc;
	}

	ep = MON_OFF2HDR(rp, rp->b_out);

	if (copy_to_user(hdr, ep, hdrbytes)) {
		mutex_unlock(&rp->fetch_lock);
		return -EFAULT;
	}

	step_len = min((unsigned int) ep->hdr.len, nbytes);
	step_len = nbytes;
	if ((offset = rp->b_out + PKT_SIZE) >= rp->b_size) offset = 0;

	if (copy_from_buf(rp, offset, data, step_len)) {
		mutex_unlock(&rp->fetch_lock);
		return -EFAULT;
	}

	spin_lock_irqsave(&rp->b_lock, flags);
	mon_buff_area_free(rp, PKT_SIZE + ep->hdr.len);
	spin_unlock_irqrestore(&rp->b_lock, flags);
	rp->b_read = 0;

	mutex_unlock(&rp->fetch_lock);
	return 0;
}

static int mon_bin_release(struct inode *inode, struct file *file)
{
	struct mon_reader_bin *rp = file->private_data;

	mon_free_buff(rp->b_vec, rp->b_size/CHUNK_SIZE);
	kfree(rp->b_vec);
	kfree(rp);

	mutex_unlock(&mon_lock);
	return 0;
}

static ssize_t mon_bin_read(struct file *file, char __user *buf,
    size_t nbytes, loff_t *ppos)
{
	struct mon_reader_bin *rp = file->private_data;
	unsigned int hdrbytes = PKT_SIZE;
	unsigned long flags;
	struct mon_bin_hdr *ep;
	unsigned int offset;
	size_t step_len;
	char *ptr;
	ssize_t done = 0;
	int rc;

	mutex_lock(&rp->fetch_lock);

	if ((rc = mon_bin_wait_event(file, rp)) < 0) {
		mutex_unlock(&rp->fetch_lock);
		return rc;
	}

	ep = MON_OFF2HDR(rp, rp->b_out);

	printk("b_out: %d;  %d < %d\n", rp->b_out, rp->b_read, hdrbytes);
	printk("%d\n", ep->hdr.len);
	if (rp->b_read < hdrbytes) {
		step_len = min(nbytes, (size_t)(hdrbytes - rp->b_read));
		ptr = ((char *)ep) + rp->b_read;
		if (step_len && copy_to_user(buf, ptr, step_len)) {
			mutex_unlock(&rp->fetch_lock);
			return -EFAULT;
		}
		nbytes -= step_len;
		buf += step_len;
		rp->b_read += step_len;
		done += step_len;
//		printk("step_len: %d nbytes %d %d\n", step_len, nbytes, (size_t)(hdrbytes - rp->b_read));
	}

	if (rp->b_read >= hdrbytes) {
		step_len = ep->hdr.len;
		step_len -= rp->b_read - hdrbytes;
		if (step_len > nbytes)
			step_len = nbytes;
		offset = rp->b_out + PKT_SIZE;
		offset += rp->b_read - hdrbytes;
		if (offset >= rp->b_size)
			offset -= rp->b_size;
		if (copy_from_buf(rp, offset, buf, step_len)) {
			mutex_unlock(&rp->fetch_lock);
			return -EFAULT;
		}
		nbytes -= step_len;
		buf += step_len;
		rp->b_read += step_len;
		done += step_len;
	}

	/*
	 * Check if whole packet was read, and if so, jump to the next one.
	 */
	if (rp->b_read >= hdrbytes + ep->hdr.len) {
		spin_lock_irqsave(&rp->b_lock, flags);
		mon_buff_area_free(rp, PKT_SIZE + ep->hdr.len);
		spin_unlock_irqrestore(&rp->b_lock, flags);
		rp->b_read = 0;
	}

	mutex_unlock(&rp->fetch_lock);
	return done;
}

/*
 * Remove at most nevents from chunked buffer.
 * Returns the number of removed events.
 */
static int mon_bin_flush(struct mon_reader_bin *rp, unsigned nevents)
{
	unsigned long flags;
	struct mon_bin_hdr *ep;
	int i;

	mutex_lock(&rp->fetch_lock);
	spin_lock_irqsave(&rp->b_lock, flags);
	for (i = 0; i < nevents; ++i) {
		if (MON_RING_EMPTY(rp))
			break;

		ep = MON_OFF2HDR(rp, rp->b_out);
		mon_buff_area_free(rp, PKT_SIZE + ep->hdr.len);
	}
	spin_unlock_irqrestore(&rp->b_lock, flags);
	rp->b_read = 0;
	mutex_unlock(&rp->fetch_lock);
	return i;
}

/*
 * Fetch at most max event offsets into the buffer and put them into vec.
 * The events are usually freed later with mon_bin_flush.
 * Return the effective number of events fetched.
 */
static int mon_bin_fetch(struct file *file, struct mon_reader_bin *rp,
    u32 __user *vec, unsigned int max)
{
	unsigned int cur_out;
	unsigned int bytes, avail;
	unsigned int size;
	unsigned int nevents;
	struct mon_bin_hdr *ep;
	unsigned long flags;
	int rc;

	mutex_lock(&rp->fetch_lock);

	if ((rc = mon_bin_wait_event(file, rp)) < 0) {
		mutex_unlock(&rp->fetch_lock);
		return rc;
	}

	spin_lock_irqsave(&rp->b_lock, flags);
	avail = rp->b_cnt;
	spin_unlock_irqrestore(&rp->b_lock, flags);

	cur_out = rp->b_out;
	nevents = 0;
	bytes = 0;
	while (bytes < avail) {
		if (nevents >= max)
			break;

		ep = MON_OFF2HDR(rp, cur_out);
		if (put_user(cur_out, &vec[nevents])) {
			mutex_unlock(&rp->fetch_lock);
			return -EFAULT;
		}

		nevents++;
		size = ep->hdr.len + PKT_SIZE;
		size = (size + PKT_ALIGN-1) & ~(PKT_ALIGN-1);
		if ((cur_out += size) >= rp->b_size)
			cur_out -= rp->b_size;
		bytes += size;
	}

	mutex_unlock(&rp->fetch_lock);
	return nevents;
}

/*
 * Count events. This is almost the same as the above mon_bin_fetch,
 * only we do not store offsets into user vector, and we have no limit.
 */
static int mon_bin_queued(struct mon_reader_bin *rp)
{
	unsigned int cur_out;
	unsigned int bytes, avail;
	unsigned int size;
	unsigned int nevents;
	struct mon_bin_hdr *ep;
	unsigned long flags;

	mutex_lock(&rp->fetch_lock);

	spin_lock_irqsave(&rp->b_lock, flags);
	avail = rp->b_cnt;
	spin_unlock_irqrestore(&rp->b_lock, flags);

	cur_out = rp->b_out;
	nevents = 0;
	bytes = 0;
	while (bytes < avail) {
		ep = MON_OFF2HDR(rp, cur_out);

		nevents++;
		size = ep->hdr.len + PKT_SIZE;
		size = (size + PKT_ALIGN-1) & ~(PKT_ALIGN-1);
		if ((cur_out += size) >= rp->b_size)
			cur_out -= rp->b_size;
		bytes += size;
	}

	mutex_unlock(&rp->fetch_lock);
	return nevents;
}

/*
 */
static long mon_bin_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct mon_reader_bin *rp = file->private_data;
	// struct mon_bus* mbus = rp->r.m_bus;
	int ret = 0;
	struct mon_bin_hdr *ep;
	unsigned long flags;

	switch (cmd) {

	case MON_IOCQ_URB_LEN:
		/*
		 * N.B. This only returns the size of data, without the header.
		 */
		spin_lock_irqsave(&rp->b_lock, flags);
		if (!MON_RING_EMPTY(rp)) {
			ep = MON_OFF2HDR(rp, rp->b_out);
			ret = ep->hdr.len;
		}
		spin_unlock_irqrestore(&rp->b_lock, flags);
		break;

	case MON_IOCQ_RING_SIZE:
		mutex_lock(&rp->fetch_lock);
		ret = rp->b_size;
		mutex_unlock(&rp->fetch_lock);
		break;

	case MON_IOCT_RING_SIZE:
		/*
		 * Changing the buffer size will flush it's contents; the new
		 * buffer is allocated before releasing the old one to be sure
		 * the device will stay functional also in case of memory
		 * pressure.
		 */
		{
		int size;
		struct mon_pgmap *vec;

		if (arg < BUFF_MIN || arg > BUFF_MAX)
			return -EINVAL;

		size = CHUNK_ALIGN(arg);
		vec = kcalloc(size / CHUNK_SIZE, sizeof(struct mon_pgmap),
			      GFP_KERNEL);
		if (vec == NULL) {
			ret = -ENOMEM;
			break;
		}

		ret = mon_alloc_buff(vec, size/CHUNK_SIZE);
		if (ret < 0) {
			kfree(vec);
			break;
		}

		mutex_lock(&rp->fetch_lock);
		spin_lock_irqsave(&rp->b_lock, flags);
		if (rp->mmap_active) {
			mon_free_buff(vec, size/CHUNK_SIZE);
			kfree(vec);
			ret = -EBUSY;
		} else {
			mon_free_buff(rp->b_vec, rp->b_size/CHUNK_SIZE);
			kfree(rp->b_vec);
			rp->b_vec  = vec;
			rp->b_size = size;
			rp->b_read = rp->b_in = rp->b_out = rp->b_cnt = 0;
			rp->cnt_lost = 0;
		}
		spin_unlock_irqrestore(&rp->b_lock, flags);
		mutex_unlock(&rp->fetch_lock);
		}
		break;

	case MON_IOCH_MFLUSH:
		ret = mon_bin_flush(rp, arg);
		break;

	case MON_IOCX_GET:
	case MON_IOCX_GETX:
		{
		struct mon_bin_get getb;

		if (copy_from_user(&getb, (void __user *)arg,
					    sizeof(struct mon_bin_get)))
			return -EFAULT;

		if (getb.alloc > 0x10000000)	/* Want to cast to u32 */
			return -EINVAL;
		ret = mon_bin_get_event(file, rp, getb.hdr,
		    PKT_SIZE,
		    getb.data, (unsigned int)getb.alloc);
		}
		break;

	case MON_IOCX_MFETCH:
		{
		struct mon_bin_mfetch mfetch;
		struct mon_bin_mfetch __user *uptr;

		uptr = (struct mon_bin_mfetch __user *)arg;

		if (copy_from_user(&mfetch, uptr, sizeof(mfetch)))
			return -EFAULT;

		if (mfetch.nflush) {
			ret = mon_bin_flush(rp, mfetch.nflush);
			if (ret < 0)
				return ret;
			if (put_user(ret, &uptr->nflush))
				return -EFAULT;
		}
		ret = mon_bin_fetch(file, rp, mfetch.offvec, mfetch.nfetch);
		if (ret < 0)
			return ret;
		if (put_user(ret, &uptr->nfetch))
			return -EFAULT;
		ret = 0;
		}
		break;

	case MON_IOCG_STATS: {
		struct mon_bin_stats __user *sp;
		unsigned int nevents;
		unsigned int ndropped;

		spin_lock_irqsave(&rp->b_lock, flags);
		ndropped = rp->cnt_lost;
		rp->cnt_lost = 0;
		spin_unlock_irqrestore(&rp->b_lock, flags);
		nevents = mon_bin_queued(rp);

		sp = (struct mon_bin_stats __user *)arg;
		if (put_user(ndropped, &sp->dropped))
			return -EFAULT;
		if (put_user(nevents, &sp->queued))
			return -EFAULT;

		}
		break;

	default:
		return -ENOTTY;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long mon_bin_compat_ioctl(struct file *file,
    unsigned int cmd, unsigned long arg)
{
	struct mon_reader_bin *rp = file->private_data;
	int ret;

	switch (cmd) {

	case MON_IOCX_GET32:
	case MON_IOCX_GETX32:
		{
		struct mon_bin_get32 getb;

		if (copy_from_user(&getb, (void __user *)arg,
					    sizeof(struct mon_bin_get32)))
			return -EFAULT;

		ret = mon_bin_get_event(file, rp, compat_ptr(getb.hdr32),
		    PKT_SIZE,
		    compat_ptr(getb.data32), getb.alloc32);
		if (ret < 0)
			return ret;
		}
		return 0;

	case MON_IOCX_MFETCH32:
		{
		struct mon_bin_mfetch32 mfetch;
		struct mon_bin_mfetch32 __user *uptr;

		uptr = (struct mon_bin_mfetch32 __user *) compat_ptr(arg);

		if (copy_from_user(&mfetch, uptr, sizeof(mfetch)))
			return -EFAULT;

		if (mfetch.nflush32) {
			ret = mon_bin_flush(rp, mfetch.nflush32);
			if (ret < 0)
				return ret;
			if (put_user(ret, &uptr->nflush32))
				return -EFAULT;
		}
		ret = mon_bin_fetch(file, rp, compat_ptr(mfetch.offvec32),
		    mfetch.nfetch32);
		if (ret < 0)
			return ret;
		if (put_user(ret, &uptr->nfetch32))
			return -EFAULT;
		}
		return 0;

	case MON_IOCG_STATS:
		return mon_bin_ioctl(file, cmd, (unsigned long) compat_ptr(arg));

	case MON_IOCQ_URB_LEN:
	case MON_IOCQ_RING_SIZE:
	case MON_IOCT_RING_SIZE:
	case MON_IOCH_MFLUSH:
		return mon_bin_ioctl(file, cmd, arg);

	default:
		;
	}
	return -ENOTTY;
}
#endif /* CONFIG_COMPAT */

static __poll_t
mon_bin_poll(struct file *file, struct poll_table_struct *wait)
{
	struct mon_reader_bin *rp = file->private_data;
	__poll_t mask = 0;
	unsigned long flags;

	if (file->f_mode & FMODE_READ)
		poll_wait(file, &rp->b_wait, wait);

	spin_lock_irqsave(&rp->b_lock, flags);
	if (!MON_RING_EMPTY(rp))
		mask |= EPOLLIN | EPOLLRDNORM;    /* readable */
	spin_unlock_irqrestore(&rp->b_lock, flags);
	return mask;
}

/*
 * open and close: just keep track of how many times the device is
 * mapped, to use the proper memory allocation function.
 */
static void mon_bin_vma_open(struct vm_area_struct *vma)
{
	struct mon_reader_bin *rp = vma->vm_private_data;
	unsigned long flags;

	spin_lock_irqsave(&rp->b_lock, flags);
	rp->mmap_active++;
	spin_unlock_irqrestore(&rp->b_lock, flags);
}

static void mon_bin_vma_close(struct vm_area_struct *vma)
{
	unsigned long flags;

	struct mon_reader_bin *rp = vma->vm_private_data;
	spin_lock_irqsave(&rp->b_lock, flags);
	rp->mmap_active--;
	spin_unlock_irqrestore(&rp->b_lock, flags);
}

/*
 * Map ring pages to user space.
 */
static vm_fault_t mon_bin_vma_fault(struct vm_fault *vmf)
{
	struct mon_reader_bin *rp = vmf->vma->vm_private_data;
	unsigned long offset, chunk_idx;
	struct page *pageptr;

	offset = vmf->pgoff << PAGE_SHIFT;
	if (offset >= rp->b_size)
		return VM_FAULT_SIGBUS;
	chunk_idx = offset / CHUNK_SIZE;
	pageptr = rp->b_vec[chunk_idx].pg;
	get_page(pageptr);
	vmf->page = pageptr;
	return 0;
}

static const struct vm_operations_struct mon_bin_vm_ops = {
	.open =     mon_bin_vma_open,
	.close =    mon_bin_vma_close,
	.fault =    mon_bin_vma_fault,
};

static int mon_bin_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/* don't do anything here: "fault" will set up page table entries */
	vma->vm_ops = &mon_bin_vm_ops;
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_private_data = filp->private_data;
	mon_bin_vma_open(vma);
	return 0;
}

static const struct file_operations mon_fops_binary = {
	.owner =	THIS_MODULE,
	.open =		mon_bin_open,
	.llseek =	no_llseek,
	.read =		mon_bin_read,
	/* .write =	mon_text_write, */
	.poll =		mon_bin_poll,
	.unlocked_ioctl = mon_bin_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl =	mon_bin_compat_ioctl,
#endif
	.release =	mon_bin_release,
	.mmap =		mon_bin_mmap,
};

static int mon_bin_wait_event(struct file *file, struct mon_reader_bin *rp)
{
	DECLARE_WAITQUEUE(waita, current);
	unsigned long flags;

	add_wait_queue(&rp->b_wait, &waita);
	set_current_state(TASK_INTERRUPTIBLE);

	spin_lock_irqsave(&rp->b_lock, flags);
	while (MON_RING_EMPTY(rp)) {
		spin_unlock_irqrestore(&rp->b_lock, flags);

		if (file->f_flags & O_NONBLOCK) {
			set_current_state(TASK_RUNNING);
			remove_wait_queue(&rp->b_wait, &waita);
			return -EWOULDBLOCK; /* Same as EAGAIN in Linux */
		}
		schedule();
		if (signal_pending(current)) {
			remove_wait_queue(&rp->b_wait, &waita);
			return -EINTR;
		}
		set_current_state(TASK_INTERRUPTIBLE);

		spin_lock_irqsave(&rp->b_lock, flags);
	}
	spin_unlock_irqrestore(&rp->b_lock, flags);

	set_current_state(TASK_RUNNING);
	remove_wait_queue(&rp->b_wait, &waita);
	return 0;
}

static int mon_alloc_buff(struct mon_pgmap *map, int npages)
{
	int n;
	unsigned long vaddr;

	for (n = 0; n < npages; n++) {
		vaddr = get_zeroed_page(GFP_KERNEL);
		if (vaddr == 0) {
			while (n-- != 0)
				free_page((unsigned long) map[n].ptr);
			return -ENOMEM;
		}
		map[n].ptr = (unsigned char *) vaddr;
		map[n].pg = virt_to_page((void *) vaddr);
	}
	return 0;
}

static void mon_free_buff(struct mon_pgmap *map, int npages)
{
	int n;

	for (n = 0; n < npages; n++)
		free_page((unsigned long) map[n].ptr);
}

typedef void (*rpmsg_trace_cb)(void*, unsigned int);
rpmsg_trace_cb tracer_cb;

void rpmsg_set_tracer(rpmsg_trace_cb cb);
void rpmsg_detach_tracer(void);


static int __init mon_init(void) {
	int rc;
	int minor = 0;

	mon_bin_class = class_create(THIS_MODULE, "rpmsgmon");
	if (IS_ERR(mon_bin_class)) {
		rc = PTR_ERR(mon_bin_class);
		goto err_class;
	}

	rc = alloc_chrdev_region(&mon_bin_dev0, 0, MON_BIN_MAX_MINOR, "rpmsgmon");
	if (rc < 0)
		goto err_dev;

	cdev_init(&mon_bin_cdev, &mon_fops_binary);
	mon_bin_cdev.owner = THIS_MODULE;

	rc = cdev_add(&mon_bin_cdev, mon_bin_dev0, MON_BIN_MAX_MINOR);
	if (rc < 0)
		goto err_add;

	
	dev = device_create(mon_bin_class, NULL,
			    MKDEV(MAJOR(mon_bin_dev0), minor), NULL,
			    "rpmsgmon%d", minor);
	if (IS_ERR(dev))
		return 0;

	printk("init");
	rpmsg_set_tracer(cb);

	return 0;

err_add:
	unregister_chrdev_region(mon_bin_dev0, MON_BIN_MAX_MINOR);
err_dev:
	class_destroy(mon_bin_class);
err_class:
	return rc;
}

static void __exit mon_exit(void) {
	printk("exit");
	rpmsg_detach_tracer();

	device_destroy(mon_bin_class, dev->devt);
	cdev_del(&mon_bin_cdev);
	unregister_chrdev_region(mon_bin_dev0, MON_BIN_MAX_MINOR);
	class_destroy(mon_bin_class);

}

module_init(mon_init);
module_exit(mon_exit);

MODULE_LICENSE("GPL");
