#ifndef __NET_NETLINK_H
#define __NET_NETLINK_H

#include <linux/types.h>
#include <linux/netlink.h>
#include <linux/jiffies.h>


enum {
	NLA_UNSPEC,
	NLA_U8,
	NLA_U16,
	NLA_U32,
	NLA_U64,
	NLA_STRING,
	NLA_FLAG,
	NLA_MSECS,
	NLA_NESTED,
	NLA_NESTED_COMPAT,
	NLA_NUL_STRING,
	NLA_BINARY,
	NLA_S8,
	NLA_S16,
	NLA_S32,
	NLA_S64,
	__NLA_TYPE_MAX,
};

#define NLA_TYPE_MAX (__NLA_TYPE_MAX - 1)

struct nla_policy {
	u16		type;
	u16		len;
};

struct nl_info {
	struct nlmsghdr		*nlh;
	struct net		*nl_net;
	u32			pid;
};

extern int		netlink_rcv_skb(struct sk_buff *skb,
					int (*cb)(struct sk_buff *,
						  struct nlmsghdr *));
extern int		nlmsg_notify(struct sock *sk, struct sk_buff *skb,
				     u32 pid, unsigned int group, int report,
				     gfp_t flags);

extern int		nla_validate(const struct nlattr *head,
				     int len, int maxtype,
				     const struct nla_policy *policy);
extern int		nla_parse(struct nlattr **tb, int maxtype,
				  const struct nlattr *head, int len,
				  const struct nla_policy *policy);
extern int		nla_policy_len(const struct nla_policy *, int);
extern struct nlattr *	nla_find(const struct nlattr *head,
				 int len, int attrtype);
extern size_t		nla_strlcpy(char *dst, const struct nlattr *nla,
				    size_t dstsize);
extern int		nla_memcpy(void *dest, const struct nlattr *src, int count);
extern int		nla_memcmp(const struct nlattr *nla, const void *data,
				   size_t size);
extern int		nla_strcmp(const struct nlattr *nla, const char *str);
extern struct nlattr *	__nla_reserve(struct sk_buff *skb, int attrtype,
				      int attrlen);
extern void *		__nla_reserve_nohdr(struct sk_buff *skb, int attrlen);
extern struct nlattr *	nla_reserve(struct sk_buff *skb, int attrtype,
				    int attrlen);
extern void *		nla_reserve_nohdr(struct sk_buff *skb, int attrlen);
extern void		__nla_put(struct sk_buff *skb, int attrtype,
				  int attrlen, const void *data);
extern void		__nla_put_nohdr(struct sk_buff *skb, int attrlen,
					const void *data);
extern int		nla_put(struct sk_buff *skb, int attrtype,
				int attrlen, const void *data);
extern int		nla_put_nohdr(struct sk_buff *skb, int attrlen,
				      const void *data);
extern int		nla_append(struct sk_buff *skb, int attrlen,
				   const void *data);


static inline int nlmsg_msg_size(int payload)
{
	return NLMSG_HDRLEN + payload;
}

static inline int nlmsg_total_size(int payload)
{
	return NLMSG_ALIGN(nlmsg_msg_size(payload));
}

static inline int nlmsg_padlen(int payload)
{
	return nlmsg_total_size(payload) - nlmsg_msg_size(payload);
}

static inline void *nlmsg_data(const struct nlmsghdr *nlh)
{
	return (unsigned char *) nlh + NLMSG_HDRLEN;
}

static inline int nlmsg_len(const struct nlmsghdr *nlh)
{
	return nlh->nlmsg_len - NLMSG_HDRLEN;
}

static inline struct nlattr *nlmsg_attrdata(const struct nlmsghdr *nlh,
					    int hdrlen)
{
	unsigned char *data = nlmsg_data(nlh);
	return (struct nlattr *) (data + NLMSG_ALIGN(hdrlen));
}

static inline int nlmsg_attrlen(const struct nlmsghdr *nlh, int hdrlen)
{
	return nlmsg_len(nlh) - NLMSG_ALIGN(hdrlen);
}

static inline int nlmsg_ok(const struct nlmsghdr *nlh, int remaining)
{
	return (remaining >= (int) sizeof(struct nlmsghdr) &&
		nlh->nlmsg_len >= sizeof(struct nlmsghdr) &&
		nlh->nlmsg_len <= remaining);
}

static inline struct nlmsghdr *
nlmsg_next(const struct nlmsghdr *nlh, int *remaining)
{
	int totlen = NLMSG_ALIGN(nlh->nlmsg_len);

	*remaining -= totlen;

	return (struct nlmsghdr *) ((unsigned char *) nlh + totlen);
}

static inline int nlmsg_parse(const struct nlmsghdr *nlh, int hdrlen,
			      struct nlattr *tb[], int maxtype,
			      const struct nla_policy *policy)
{
	if (nlh->nlmsg_len < nlmsg_msg_size(hdrlen))
		return -EINVAL;

	return nla_parse(tb, maxtype, nlmsg_attrdata(nlh, hdrlen),
			 nlmsg_attrlen(nlh, hdrlen), policy);
}

static inline struct nlattr *nlmsg_find_attr(const struct nlmsghdr *nlh,
					     int hdrlen, int attrtype)
{
	return nla_find(nlmsg_attrdata(nlh, hdrlen),
			nlmsg_attrlen(nlh, hdrlen), attrtype);
}

static inline int nlmsg_validate(const struct nlmsghdr *nlh,
				 int hdrlen, int maxtype,
				 const struct nla_policy *policy)
{
	if (nlh->nlmsg_len < nlmsg_msg_size(hdrlen))
		return -EINVAL;

	return nla_validate(nlmsg_attrdata(nlh, hdrlen),
			    nlmsg_attrlen(nlh, hdrlen), maxtype, policy);
}

static inline int nlmsg_report(const struct nlmsghdr *nlh)
{
	return !!(nlh->nlmsg_flags & NLM_F_ECHO);
}

#define nlmsg_for_each_attr(pos, nlh, hdrlen, rem) \
	nla_for_each_attr(pos, nlmsg_attrdata(nlh, hdrlen), \
			  nlmsg_attrlen(nlh, hdrlen), rem)

static inline struct nlmsghdr *nlmsg_put(struct sk_buff *skb, u32 pid, u32 seq,
					 int type, int payload, int flags)
{
	if (unlikely(skb_tailroom(skb) < nlmsg_total_size(payload)))
		return NULL;

	return __nlmsg_put(skb, pid, seq, type, payload, flags);
}

static inline struct nlmsghdr *nlmsg_put_answer(struct sk_buff *skb,
						struct netlink_callback *cb,
						int type, int payload,
						int flags)
{
	return nlmsg_put(skb, NETLINK_CB(cb->skb).pid, cb->nlh->nlmsg_seq,
			 type, payload, flags);
}

static inline struct sk_buff *nlmsg_new(size_t payload, gfp_t flags)
{
	return alloc_skb(nlmsg_total_size(payload), flags);
}

static inline int nlmsg_end(struct sk_buff *skb, struct nlmsghdr *nlh)
{
	nlh->nlmsg_len = skb_tail_pointer(skb) - (unsigned char *)nlh;

	return skb->len;
}

static inline void *nlmsg_get_pos(struct sk_buff *skb)
{
	return skb_tail_pointer(skb);
}

static inline void nlmsg_trim(struct sk_buff *skb, const void *mark)
{
	if (mark)
		skb_trim(skb, (unsigned char *) mark - skb->data);
}

static inline void nlmsg_cancel(struct sk_buff *skb, struct nlmsghdr *nlh)
{
	nlmsg_trim(skb, nlh);
}

static inline void nlmsg_free(struct sk_buff *skb)
{
	kfree_skb(skb);
}

static inline int nlmsg_multicast(struct sock *sk, struct sk_buff *skb,
				  u32 pid, unsigned int group, gfp_t flags)
{
	int err;

	NETLINK_CB(skb).dst_group = group;

	err = netlink_broadcast(sk, skb, pid, group, flags);
	if (err > 0)
		err = 0;

	return err;
}

static inline int nlmsg_unicast(struct sock *sk, struct sk_buff *skb, u32 pid)
{
	int err;

	err = netlink_unicast(sk, skb, pid, MSG_DONTWAIT);
	if (err > 0)
		err = 0;

	return err;
}

#define nlmsg_for_each_msg(pos, head, len, rem) \
	for (pos = head, rem = len; \
	     nlmsg_ok(pos, rem); \
	     pos = nlmsg_next(pos, &(rem)))

static inline void
nl_dump_check_consistent(struct netlink_callback *cb,
			 struct nlmsghdr *nlh)
{
	if (cb->prev_seq && cb->seq != cb->prev_seq)
		nlh->nlmsg_flags |= NLM_F_DUMP_INTR;
	cb->prev_seq = cb->seq;
}


static inline int nla_attr_size(int payload)
{
	return NLA_HDRLEN + payload;
}

static inline int nla_total_size(int payload)
{
	return NLA_ALIGN(nla_attr_size(payload));
}

static inline int nla_padlen(int payload)
{
	return nla_total_size(payload) - nla_attr_size(payload);
}

static inline int nla_type(const struct nlattr *nla)
{
	return nla->nla_type & NLA_TYPE_MASK;
}

static inline void *nla_data(const struct nlattr *nla)
{
	return (char *) nla + NLA_HDRLEN;
}

static inline int nla_len(const struct nlattr *nla)
{
	return nla->nla_len - NLA_HDRLEN;
}

static inline int nla_ok(const struct nlattr *nla, int remaining)
{
	return remaining >= (int) sizeof(*nla) &&
	       nla->nla_len >= sizeof(*nla) &&
	       nla->nla_len <= remaining;
}

static inline struct nlattr *nla_next(const struct nlattr *nla, int *remaining)
{
	int totlen = NLA_ALIGN(nla->nla_len);

	*remaining -= totlen;
	return (struct nlattr *) ((char *) nla + totlen);
}

static inline struct nlattr *
nla_find_nested(const struct nlattr *nla, int attrtype)
{
	return nla_find(nla_data(nla), nla_len(nla), attrtype);
}

static inline int nla_parse_nested(struct nlattr *tb[], int maxtype,
				   const struct nlattr *nla,
				   const struct nla_policy *policy)
{
	return nla_parse(tb, maxtype, nla_data(nla), nla_len(nla), policy);
}

static inline int nla_put_u8(struct sk_buff *skb, int attrtype, u8 value)
{
	return nla_put(skb, attrtype, sizeof(u8), &value);
}

static inline int nla_put_u16(struct sk_buff *skb, int attrtype, u16 value)
{
	return nla_put(skb, attrtype, sizeof(u16), &value);
}

static inline int nla_put_u32(struct sk_buff *skb, int attrtype, u32 value)
{
	return nla_put(skb, attrtype, sizeof(u32), &value);
}

static inline int nla_put_u64(struct sk_buff *skb, int attrtype, u64 value)
{
	return nla_put(skb, attrtype, sizeof(u64), &value);
}

static inline int nla_put_s8(struct sk_buff *skb, int attrtype, s8 value)
{
	return nla_put(skb, attrtype, sizeof(s8), &value);
}

static inline int nla_put_s16(struct sk_buff *skb, int attrtype, s16 value)
{
	return nla_put(skb, attrtype, sizeof(s16), &value);
}

static inline int nla_put_s32(struct sk_buff *skb, int attrtype, s32 value)
{
	return nla_put(skb, attrtype, sizeof(s32), &value);
}

static inline int nla_put_s64(struct sk_buff *skb, int attrtype, s64 value)
{
	return nla_put(skb, attrtype, sizeof(s64), &value);
}

static inline int nla_put_string(struct sk_buff *skb, int attrtype,
				 const char *str)
{
	return nla_put(skb, attrtype, strlen(str) + 1, str);
}

static inline int nla_put_flag(struct sk_buff *skb, int attrtype)
{
	return nla_put(skb, attrtype, 0, NULL);
}

static inline int nla_put_msecs(struct sk_buff *skb, int attrtype,
				unsigned long jiffies)
{
	u64 tmp = jiffies_to_msecs(jiffies);
	return nla_put(skb, attrtype, sizeof(u64), &tmp);
}

#define NLA_PUT(skb, attrtype, attrlen, data) \
	do { \
		if (unlikely(nla_put(skb, attrtype, attrlen, data) < 0)) \
			goto nla_put_failure; \
	} while(0)

#define NLA_PUT_TYPE(skb, type, attrtype, value) \
	do { \
		type __tmp = value; \
		NLA_PUT(skb, attrtype, sizeof(type), &__tmp); \
	} while(0)

#define NLA_PUT_U8(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, u8, attrtype, value)

#define NLA_PUT_U16(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, u16, attrtype, value)

#define NLA_PUT_LE16(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, __le16, attrtype, value)

#define NLA_PUT_BE16(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, __be16, attrtype, value)

#define NLA_PUT_NET16(skb, attrtype, value) \
	NLA_PUT_BE16(skb, attrtype | NLA_F_NET_BYTEORDER, value)

#define NLA_PUT_U32(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, u32, attrtype, value)

#define NLA_PUT_BE32(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, __be32, attrtype, value)

#define NLA_PUT_NET32(skb, attrtype, value) \
	NLA_PUT_BE32(skb, attrtype | NLA_F_NET_BYTEORDER, value)

#define NLA_PUT_U64(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, u64, attrtype, value)

#define NLA_PUT_BE64(skb, attrtype, value) \
	NLA_PUT_TYPE(skb, __be64, attrtype, value)

#define NLA_PUT_NET64(skb, attrtype, value) \
	NLA_PUT_BE64(skb, attrtype | NLA_F_NET_BYTEORDER, value)

#define NLA_PUT_STRING(skb, attrtype, value) \
	NLA_PUT(skb, attrtype, strlen(value) + 1, value)

#define NLA_PUT_FLAG(skb, attrtype) \
	NLA_PUT(skb, attrtype, 0, NULL)

#define NLA_PUT_MSECS(skb, attrtype, jiffies) \
	NLA_PUT_U64(skb, attrtype, jiffies_to_msecs(jiffies))

static inline u32 nla_get_u32(const struct nlattr *nla)
{
	return *(u32 *) nla_data(nla);
}

static inline __be32 nla_get_be32(const struct nlattr *nla)
{
	return *(__be32 *) nla_data(nla);
}

static inline u16 nla_get_u16(const struct nlattr *nla)
{
	return *(u16 *) nla_data(nla);
}

static inline __be16 nla_get_be16(const struct nlattr *nla)
{
	return *(__be16 *) nla_data(nla);
}

static inline __le16 nla_get_le16(const struct nlattr *nla)
{
	return *(__le16 *) nla_data(nla);
}

static inline u8 nla_get_u8(const struct nlattr *nla)
{
	return *(u8 *) nla_data(nla);
}

static inline u64 nla_get_u64(const struct nlattr *nla)
{
	u64 tmp;

	nla_memcpy(&tmp, nla, sizeof(tmp));

	return tmp;
}

static inline __be64 nla_get_be64(const struct nlattr *nla)
{
	__be64 tmp;

	nla_memcpy(&tmp, nla, sizeof(tmp));

	return tmp;
}

static inline s32 nla_get_s32(const struct nlattr *nla)
{
	return *(s32 *) nla_data(nla);
}

static inline s16 nla_get_s16(const struct nlattr *nla)
{
	return *(s16 *) nla_data(nla);
}

static inline s8 nla_get_s8(const struct nlattr *nla)
{
	return *(s8 *) nla_data(nla);
}

static inline s64 nla_get_s64(const struct nlattr *nla)
{
	s64 tmp;

	nla_memcpy(&tmp, nla, sizeof(tmp));

	return tmp;
}

static inline int nla_get_flag(const struct nlattr *nla)
{
	return !!nla;
}

static inline unsigned long nla_get_msecs(const struct nlattr *nla)
{
	u64 msecs = nla_get_u64(nla);

	return msecs_to_jiffies((unsigned long) msecs);
}

static inline struct nlattr *nla_nest_start(struct sk_buff *skb, int attrtype)
{
	struct nlattr *start = (struct nlattr *)skb_tail_pointer(skb);

	if (nla_put(skb, attrtype, 0, NULL) < 0)
		return NULL;

	return start;
}

static inline int nla_nest_end(struct sk_buff *skb, struct nlattr *start)
{
	start->nla_len = skb_tail_pointer(skb) - (unsigned char *)start;
	return skb->len;
}

static inline void nla_nest_cancel(struct sk_buff *skb, struct nlattr *start)
{
	nlmsg_trim(skb, start);
}

static inline int nla_validate_nested(const struct nlattr *start, int maxtype,
				      const struct nla_policy *policy)
{
	return nla_validate(nla_data(start), nla_len(start), maxtype, policy);
}

#define nla_for_each_attr(pos, head, len, rem) \
	for (pos = head, rem = len; \
	     nla_ok(pos, rem); \
	     pos = nla_next(pos, &(rem)))

#define nla_for_each_nested(pos, nla, rem) \
	nla_for_each_attr(pos, nla_data(nla), nla_len(nla), rem)

#endif
