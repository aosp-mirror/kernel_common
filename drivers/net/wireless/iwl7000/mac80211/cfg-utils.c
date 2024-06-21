/*
 * Wireless utility functions
 *
 * Copyright 2007-2009	Johannes Berg <johannes@sipsolutions.net>
 * Copyright 2013-2014  Intel Mobile Communications GmbH
 * Copyright (C) 2018-2020, 2022-2024 Intel Corporation
 */
#include <linux/export.h>
#include <net/cfg80211.h>

#if CFG80211_VERSION < KERNEL_VERSION(5,6,0)
int ieee80211_get_vht_max_nss(struct ieee80211_vht_cap *cap,
			      enum ieee80211_vht_chanwidth bw,
			      int mcs, bool ext_nss_bw_capable,
			      unsigned int max_vht_nss)
{
	u16 map = le16_to_cpu(cap->supp_mcs.rx_mcs_map);
	int ext_nss_bw;
	int supp_width;
	int i, mcs_encoding;

	if (map == 0xffff)
		return 0;

	if (WARN_ON(mcs > 9))
		return 0;
	if (mcs <= 7)
		mcs_encoding = 0;
	else if (mcs == 8)
		mcs_encoding = 1;
	else
		mcs_encoding = 2;

	if (!max_vht_nss) {
		/* find max_vht_nss for the given MCS */
		for (i = 7; i >= 0; i--) {
			int supp = (map >> (2 * i)) & 3;

			if (supp == 3)
				continue;

			if (supp >= mcs_encoding) {
				max_vht_nss = i + 1;
				break;
			}
		}
	}

	if (!(cap->supp_mcs.tx_mcs_map &
			cpu_to_le16(IEEE80211_VHT_EXT_NSS_BW_CAPABLE)))
		return max_vht_nss;

	ext_nss_bw = le32_get_bits(cap->vht_cap_info,
				   IEEE80211_VHT_CAP_EXT_NSS_BW_MASK);
	supp_width = le32_get_bits(cap->vht_cap_info,
				   IEEE80211_VHT_CAP_SUPP_CHAN_WIDTH_MASK);

	/* if not capable, treat ext_nss_bw as 0 */
	if (!ext_nss_bw_capable)
		ext_nss_bw = 0;

	/* This is invalid */
	if (supp_width == 3)
		return 0;

	/* This is an invalid combination so pretend nothing is supported */
	if (supp_width == 2 && (ext_nss_bw == 1 || ext_nss_bw == 2))
		return 0;

	/*
	 * Cover all the special cases according to IEEE 802.11-2016
	 * Table 9-250. All other cases are either factor of 1 or not
	 * valid/supported.
	 */
	switch (bw) {
	case IEEE80211_VHT_CHANWIDTH_USE_HT:
	case IEEE80211_VHT_CHANWIDTH_80MHZ:
		if ((supp_width == 1 || supp_width == 2) &&
		    ext_nss_bw == 3)
			return 2 * max_vht_nss;
		break;
	case IEEE80211_VHT_CHANWIDTH_160MHZ:
		if (supp_width == 0 &&
		    (ext_nss_bw == 1 || ext_nss_bw == 2))
			return max_vht_nss / 2;
		if (supp_width == 0 &&
		    ext_nss_bw == 3)
			return (3 * max_vht_nss) / 4;
		if (supp_width == 1 &&
		    ext_nss_bw == 3)
			return 2 * max_vht_nss;
		break;
	case IEEE80211_VHT_CHANWIDTH_80P80MHZ:
		if (supp_width == 0 && ext_nss_bw == 1)
			return 0; /* not possible */
		if (supp_width == 0 &&
		    ext_nss_bw == 2)
			return max_vht_nss / 2;
		if (supp_width == 0 &&
		    ext_nss_bw == 3)
			return (3 * max_vht_nss) / 4;
		if (supp_width == 1 &&
		    ext_nss_bw == 0)
			return 0; /* not possible */
		if (supp_width == 1 &&
		    ext_nss_bw == 1)
			return max_vht_nss / 2;
		if (supp_width == 1 &&
		    ext_nss_bw == 2)
			return (3 * max_vht_nss) / 4;
		break;
	}

	/* not covered or invalid combination received */
	return max_vht_nss;
}
EXPORT_SYMBOL(ieee80211_get_vht_max_nss);
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,9,0)
bool cfg80211_iter_rnr(const u8 *elems, size_t elems_len,
		       enum cfg80211_rnr_iter_ret
		       (*iter)(void *data, u8 type,
			       const struct ieee80211_neighbor_ap_info *info,
			       const u8 *tbtt_info, u8 tbtt_info_len),
		       void *iter_data)
{
	const struct element *rnr;
	const u8 *pos, *end;

	for_each_element_id(rnr, WLAN_EID_REDUCED_NEIGHBOR_REPORT,
			    elems, elems_len) {
		const struct ieee80211_neighbor_ap_info *info;

		pos = rnr->data;
		end = rnr->data + rnr->datalen;

		/* RNR IE may contain more than one NEIGHBOR_AP_INFO */
		while (sizeof(*info) <= end - pos) {
			u8 length, i, count;
			u8 type;

			info = (void *)pos;
			count = u8_get_bits(info->tbtt_info_hdr,
					    IEEE80211_AP_INFO_TBTT_HDR_COUNT) +
				1;
			length = info->tbtt_info_len;

			pos += sizeof(*info);

			if (count * length > end - pos)
				return false;

			type = u8_get_bits(info->tbtt_info_hdr,
					   IEEE80211_AP_INFO_TBTT_HDR_TYPE);

			for (i = 0; i < count; i++) {
				switch (iter(iter_data, type, info,
					     pos, length)) {
				case RNR_ITER_CONTINUE:
					break;
				case RNR_ITER_BREAK:
					return true;
				case RNR_ITER_ERROR:
					return false;
				}

				pos += length;
			}
		}

		if (pos != end)
			return false;
	}

	return true;
}

ssize_t cfg80211_defragment_element(const struct element *elem, const u8 *ies,
				    size_t ieslen, u8 *data, size_t data_len,
				    u8 frag_id)
{
	const struct element *next;
	ssize_t copied;
	u8 elem_datalen;

	if (!elem)
		return -EINVAL;

	/* elem might be invalid after the memmove */
	next = (void *)(elem->data + elem->datalen);
	elem_datalen = elem->datalen;

	if (elem->id == WLAN_EID_EXTENSION) {
		copied = elem->datalen - 1;

		if (data) {
			if (copied > data_len)
				return -ENOSPC;

			memmove(data, elem->data + 1, copied);
		}
	} else {
		copied = elem->datalen;

		if (data) {
			if (copied > data_len)
				return -ENOSPC;

			memmove(data, elem->data, copied);
		}
	}

	/* Fragmented elements must have 255 bytes */
	if (elem_datalen < 255)
		return copied;

	for (elem = next;
	     elem->data < ies + ieslen &&
		elem->data + elem->datalen <= ies + ieslen;
	     elem = next) {
		/* elem might be invalid after the memmove */
		next = (void *)(elem->data + elem->datalen);

		if (elem->id != frag_id)
			break;

		elem_datalen = elem->datalen;

		if (data) {
			if (copied + elem_datalen > data_len)
				return -ENOSPC;

			memmove(data + copied, elem->data, elem_datalen);
		}

		copied += elem_datalen;

		/* Only the last fragment may be short */
		if (elem_datalen != 255)
			break;
	}

	return copied;
}
EXPORT_SYMBOL(cfg80211_defragment_element);
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,7,0)
void ieee80211_fragment_element(struct sk_buff *skb, u8 *len_pos, u8 frag_id)
{
	unsigned int elem_len;

	if (!len_pos)
		return;

	elem_len = skb->data + skb->len - len_pos - 1;

	while (elem_len > 255) {
		/* this one is 255 */
		*len_pos = 255;
		/* remaining data gets smaller */
		elem_len -= 255;
		/* make space for the fragment ID/len in SKB */
		skb_put(skb, 2);
		/* shift back the remaining data to place fragment ID/len */
		memmove(len_pos + 255 + 3, len_pos + 255 + 1, elem_len);
		/* place the fragment ID */
		len_pos += 255 + 1;
		*len_pos = frag_id;
		/* and point to fragment length to update later */
		len_pos++;
	}

	*len_pos = elem_len;
}
EXPORT_SYMBOL(ieee80211_fragment_element);
#endif

#if CFG80211_VERSION <= KERNEL_VERSION(6,8,0)
bool
ieee80211_uhb_power_type_valid(struct ieee80211_mgmt *mgmt, size_t len,
			       struct ieee80211_channel *channel)
{
	const struct element *tmp;
	struct ieee80211_he_operation *he_oper;
	bool ret = false;
	size_t ielen, min_hdr_len;
	u8 *variable = mgmt->u.probe_resp.variable;

	min_hdr_len = offsetof(struct ieee80211_mgmt,
			       u.probe_resp.variable);
	ielen = len - min_hdr_len;

	if (channel->band != NL80211_BAND_6GHZ)
		return true;

	tmp = cfg80211_find_ext_elem(WLAN_EID_EXT_HE_OPERATION,
				     variable, ielen);
	if (tmp && tmp->datalen >= sizeof(*he_oper) + 1) {
		const struct ieee80211_he_6ghz_oper *he_6ghz_oper;

		he_oper = (void *)&tmp->data[1];
		he_6ghz_oper = ieee80211_he_6ghz_oper(he_oper);
		if (!he_6ghz_oper)
			return false;

		switch (u8_get_bits(he_6ghz_oper->control,
				    IEEE80211_HE_6GHZ_OPER_CTRL_REG_INFO)) {
		case IEEE80211_6GHZ_CTRL_REG_LPI_AP:
		case IEEE80211_6GHZ_CTRL_REG_INDOOR_LPI_AP:
			return true;
		case IEEE80211_6GHZ_CTRL_REG_SP_AP:
		case IEEE80211_6GHZ_CTRL_REG_INDOOR_SP_AP:
			return !(channel->flags &
				 IEEE80211_CHAN_NO_6GHZ_AFC_CLIENT);
		case IEEE80211_6GHZ_CTRL_REG_VLP_AP:
			return !(channel->flags &
				 IEEE80211_CHAN_NO_6GHZ_VLP_CLIENT);
		default:
			return false;
		}
	}
	return false;
}
#endif

#if CFG80211_VERSION < KERNEL_VERSION(6,9,0)
bool ieee80211_operating_class_to_chandef(u8 operating_class,
					  struct ieee80211_channel *chan,
					  struct cfg80211_chan_def *chandef)
{
	u32 control_freq, offset = 0;
	enum nl80211_band band;

	if (!ieee80211_operating_class_to_band(operating_class, &band) ||
	    !chan || band != chan->band)
		return false;

	control_freq = chan->center_freq;
	chandef->chan = chan;

	if (control_freq >= 5955)
		offset = control_freq - 5955;
	else if (control_freq >= 5745)
		offset = control_freq - 5745;
	else if (control_freq >= 5180)
		offset = control_freq - 5180;
	offset /= 20;

	switch (operating_class) {
	case 81:  /* 2 GHz band; 20 MHz; channels 1..13 */
	case 82:  /* 2 GHz band; 20 MHz; channel 14 */
	case 115: /* 5 GHz band; 20 MHz; channels 36,40,44,48 */
	case 118: /* 5 GHz band; 20 MHz; channels 52,56,60,64 */
	case 121: /* 5 GHz band; 20 MHz; channels 100..144 */
	case 124: /* 5 GHz band; 20 MHz; channels 149,153,157,161 */
	case 125: /* 5 GHz band; 20 MHz; channels 149..177 */
	case 131: /* 6 GHz band; 20 MHz; channels 1..233*/
	case 136: /* 6 GHz band; 20 MHz; channel 2 */
		chandef->center_freq1 = control_freq;
		chandef->width = NL80211_CHAN_WIDTH_20;
		return true;
	case 83:  /* 2 GHz band; 40 MHz; channels 1..9 */
	case 116: /* 5 GHz band; 40 MHz; channels 36,44 */
	case 119: /* 5 GHz band; 40 MHz; channels 52,60 */
	case 122: /* 5 GHz band; 40 MHz; channels 100,108,116,124,132,140 */
	case 126: /* 5 GHz band; 40 MHz; channels 149,157,165,173 */
		chandef->center_freq1 = control_freq + 10;
		chandef->width = NL80211_CHAN_WIDTH_40;
		return true;
	case 84:  /* 2 GHz band; 40 MHz; channels 5..13 */
	case 117: /* 5 GHz band; 40 MHz; channels 40,48 */
	case 120: /* 5 GHz band; 40 MHz; channels 56,64 */
	case 123: /* 5 GHz band; 40 MHz; channels 104,112,120,128,136,144 */
	case 127: /* 5 GHz band; 40 MHz; channels 153,161,169,177 */
		chandef->center_freq1 = control_freq - 10;
		chandef->width = NL80211_CHAN_WIDTH_40;
		return true;
	case 132: /* 6 GHz band; 40 MHz; channels 1,5,..,229*/
		chandef->center_freq1 = control_freq + 10 - (offset & 1) * 20;
		chandef->width = NL80211_CHAN_WIDTH_40;
		return true;
	case 128: /* 5 GHz band; 80 MHz; channels 36..64,100..144,149..177 */
	case 133: /* 6 GHz band; 80 MHz; channels 1,5,..,229 */
		chandef->center_freq1 = control_freq + 30 - (offset & 3) * 20;
		chandef->width = NL80211_CHAN_WIDTH_80;
		return true;
	case 129: /* 5 GHz band; 160 MHz; channels 36..64,100..144,149..177 */
	case 134: /* 6 GHz band; 160 MHz; channels 1,5,..,229 */
		chandef->center_freq1 = control_freq + 70 - (offset & 7) * 20;
		chandef->width = NL80211_CHAN_WIDTH_160;
		return true;
	case 130: /* 5 GHz band; 80+80 MHz; channels 36..64,100..144,149..177 */
	case 135: /* 6 GHz band; 80+80 MHz; channels 1,5,..,229 */
		  /* The center_freq2 of 80+80 MHz is unknown */
	case 137: /* 6 GHz band; 320 MHz; channels 1,5,..,229 */
		  /* 320-1 or 320-2 channelization is unknown */
	default:
		return false;
	}
}
#endif
