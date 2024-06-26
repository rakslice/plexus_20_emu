#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "scsi.h"
#include "emu.h"
#include "log.h"

// Debug logging
#define SCSI_LOG(msg_level, format_and_args...) \
        log_printf(LOG_SRC_SCSI, msg_level, format_and_args)
#define SCSI_LOG_DEBUG(format_and_args...)  SCSI_LOG(LOG_DEBUG,  format_and_args)
#define SCSI_LOG_INFO(format_and_args...)   SCSI_LOG(LOG_INFO,   format_and_args)
#define SCSI_LOG_NOTICE(format_and_args...) SCSI_LOG(LOG_NOTICE, format_and_args)

typedef struct {
	scsi_dev_t dev;
	FILE *hdfile;
	int writes_enabled;
	uint8_t cmd[10];
	int result;
} scsi_hd_t;


static const uint8_t sense[]={
	0x80+0x00, //error code
	0, //sense key
	0,0,0, //additional information
	0, //additional sense length
	0,0,0,0, //cmd specific info
	0,	//asc
	0,	//ascq
	0,	//fru code
	0,0,0,0	//sense key specific
};
static const int sense_template_len=17;

static int hd_handle_cmd(scsi_dev_t *dev, uint8_t *cd, int len) {
	scsi_hd_t *hd=(scsi_hd_t*)dev;
	if (len<6 || len>10) return SCSI_DEV_ERR;
	memcpy(hd->cmd, cd, len);
	if (cd[0]==0) {
		return SCSI_DEV_STATUS;
	} else if (cd[0]==1) {
		return SCSI_DEV_STATUS;
	} else if (cd[0]==3) { //request sense
		return SCSI_DEV_DATA_IN;
	} else if (cd[0]==8) {
		return SCSI_DEV_DATA_IN;
	} else if (cd[0]==0x15) { //mode select
		return SCSI_DEV_DATA_OUT;
	} else if (cd[0]==0xa) { //write
		return SCSI_DEV_DATA_OUT;
	} else if (cd[0]==0xC2) {
		//omti config cmd
		return SCSI_DEV_DATA_OUT;
	} else {
		SCSI_LOG_NOTICE("hd: unsupported cmd %d\n", cd[0]);
		exit(1);
	}
	return SCSI_DEV_DATA_IN;
}

#define SENSE_WRITE_FAULT 0x3
#define SENSE_DRIVE_NOT_READY 0x4
#define SENSE_SEEK_ERROR 0x15
#define SENSE_WRITE_PROTECTED 0x17
#define SENSE_INVALID_CMD 0x20
#define SENSE_ILLEGAL_PARAMS 0x21
#define SENSE_VOLUME_OVERFLOW 0x23

int hd_handle_data_in(scsi_dev_t *dev, uint8_t *msg, int buflen) {
	scsi_hd_t *hd=(scsi_hd_t*)dev;
	if (hd->cmd[0]==3) { //sense
		int clen=hd->cmd[4];
		if (clen==0) clen=4; //per scsi spec
		int lun=(hd->cmd[1]&0x60)>>5;
		if (clen>buflen) clen=buflen;
		if (clen>sense_template_len) clen=sense_template_len;
		memcpy(msg, sense, clen);

		if (lun != 0) {
			SCSI_LOG_DEBUG("SCSI HDFL: lun %d sense, let's give drive not ready\n", lun);
			msg[0] = SENSE_DRIVE_NOT_READY;
			msg[1] |= lun<<5;
		} else {
			if (hd->result != 0) {
				// there's a result from a previous operation
				msg[0] = hd->result;
				msg[1] |= lun<<5;
			}
		}
		return clen;
	} else if (hd->cmd[0]==8) { //read
		int first_reserved=hd->cmd[1]&0x80;
		int lun=(hd->cmd[1]&0x60)>>5;
		int lba=((hd->cmd[1]&0x1f)<<16)+(hd->cmd[2]<<8)+(hd->cmd[3]);
		int tlen=hd->cmd[4]; //note 0 means 256 blocks...
		if (tlen == 0) {
			SCSI_LOG_DEBUG("SCSI HD: tlen 0->256\n");
			tlen = 256;
		}
		if (first_reserved != 0) {
			SCSI_LOG_DEBUG("SCSI HDFL: lun %d read cmd %d sectors at lba %d: byte 1 reserved bits set: 0x%x\n", lun, tlen, lba, first_reserved);
		} else {
			SCSI_LOG_DEBUG("SCSI HDFL: lun %d read cmd %d sectors at lba %d\n", lun, tlen, lba);
		}

		int blen=tlen*512;
		if (blen>buflen) blen=buflen;

		if (lun!=0) {
			SCSI_LOG_DEBUG("SCSI HDFL: read cmd giving placeholder data for unavailable lun %d\n", lun);
			for (int i = 0; i < blen; i++)
				msg[i] = 0;
			return blen;
		}

		if (fseek(hd->hdfile, lba*512, SEEK_SET)!=0) {
			SCSI_LOG_NOTICE("Seek to lba %d failed\n", lba);
			exit(1);
		};
		int read_result = fread(msg, blen, 1, hd->hdfile);
		if (read_result != 1) {
			SCSI_LOG_NOTICE("Read of %d sectors at %d failed, got %d\n", tlen, lba, read_result);
			exit(1);
		}
//		SCSI_LOG_DEBUG("Read %d bytes from LB %d\n", blen, lba);
		return blen;
	} else {
		SCSI_LOG_NOTICE("hd: unsupported hd_handle_data_in 0x%x\n", hd->cmd[0]);
		printf("Unknown command: 0x%x\n", hd->cmd[0]);
		assert(0 && "hd_handle_data_in: unknown cmd");
	}
	return 0;
}

static void hd_handle_data_out(scsi_dev_t *dev, uint8_t *msg, int buflen) {
	scsi_hd_t *hd=(scsi_hd_t*)dev;
	hd->result=0xff;
	if (hd->cmd[0]==0x15) { //mode select
		//ignore
		hd->result=0;
	} else if (hd->cmd[0]==0x0a) { //write
		int lun=(hd->cmd[1]&0x60)>>5;
		SCSI_LOG_DEBUG("hd: write(6), lun %d, msg size %d\n", lun, buflen);
		int lba=((hd->cmd[1]&0x1f)<<16)+(hd->cmd[2]<<8)+(hd->cmd[3]);
		int tlen=hd->cmd[4]; //note 0 means 256 blocks...
		if (tlen == 0) {
			SCSI_LOG_DEBUG("SCSI HD: tlen 0->256\n");
			tlen = 256;
		}
		SCSI_LOG_DEBUG("hd: lba %d, tlen %d\n", lba, tlen);

		int blen=tlen*512;
		if (blen>buflen) blen=buflen;

		SCSI_LOG_DEBUG("hd: data bytes to write %d\n", blen);

		assert(blen==buflen);

		if (lun!=0) {
			SCSI_LOG_DEBUG("SCSI HDFL: write for unavailable lun %d\n erroring\n", lun);
			hd->result=1;
			return;
		}

		if (!hd->writes_enabled) {
			SCSI_LOG_DEBUG("SCSI HDML: lun %d: hd is write protected\n", lun);
			hd->result=SENSE_WRITE_PROTECTED;
			return;
		}

		if (fseek(hd->hdfile, lba*512, SEEK_SET)!=0) {
			SCSI_LOG_NOTICE("SCSI HDFL: lun %d: seek to lba %d failed\n", lun, lba);
			hd->result=SENSE_SEEK_ERROR;
			return;
		};
		int write_result = fwrite(msg, buflen, 1, hd->hdfile);
		if (write_result != 1) {
			SCSI_LOG_NOTICE("SCSI HDFL: lun %d: Write of %d sectors at %d failed, got %d\n", lun, tlen, lba, write_result);
			hd->result=SENSE_WRITE_FAULT;
			return;
		}
		SCSI_LOG_DEBUG("Wrote %d bytes to LB %d\n", blen, lba);
		hd->result=0;
	} else if (hd->cmd[0]==0xc2) { //omti disk params
		int lun=(hd->cmd[1]&0x60)>>5;
		SCSI_LOG_DEBUG("hd: omti disk parameters for lun %d\n", lun);
		if (buflen<10) {
			SCSI_LOG_DEBUG("hd: truncated omti msg len %d\n", buflen);
			return;
		}
		int spw=msg[0];
		int sp=msg[1];
		int sm=msg[2];
		int heads=msg[3];
		int cyl=msg[4]<<8|msg[5];
		int wp=(msg[7]&3)<<8|msg[6];
		SCSI_LOG_DEBUG("hd: spw %d, sp %d, sm %d, heads %d, cyl %d, wp/wsi %d\n",
			spw, sp, sm, heads, cyl, wp);
		int type=(msg[7]>>4)&3;

		char * types[] = {"fixed", "unknown val", "fixed/removable", "removable"};
		int hs = msg[7]&8;
		int spt = msg[8];
		SCSI_LOG_DEBUG("hd: type %s, sectors %s, sectors per track %d\n", types[type], hs? "hard":"soft", spt);
		hd->result=0;
	} else {
		SCSI_LOG_NOTICE("hd: unsupported hd_handle_data_out 0x%x\n", hd->cmd[0]);
		exit(1);
	}
}

static int hd_handle_status(scsi_dev_t *dev) {
	scsi_hd_t *hd=(scsi_hd_t*)dev;
	if (hd->cmd[0]==0) {
		int lun=(hd->cmd[1]&0x60)>>5;
		if (lun != 0) {
			SCSI_LOG_DEBUG("SCSI HDFL: lun %d test, lun not available; check condition\n", lun);
			return (lun<<5)|2; // check condition with lun
		}

		return 0;
	}
	else if (hd->cmd[0]==1) { // recalibrate;
		return 0;
	}
	else if (hd->cmd[0]==0xc2) { // omti disk config
		return 0;
	}
	else if (hd->cmd[0]==3) { // request sense;
		int lun=(hd->cmd[1]&0x60)>>5;
		SCSI_LOG_DEBUG("SCSI HDFL: lun %d request sense\n", lun);
		return 0;
	}
	else if (hd->cmd[0]==8) { // read
		int lun=(hd->cmd[1]&0x60)>>5;
		if (lun != 0) {
			SCSI_LOG_DEBUG("SCSI HDFL: lun %d drive not available; check condition\n", lun);
			return (lun<<5)|2; // check condition with lun
		}
	}
	else if (hd->cmd[0]==0x0a) { // write
		int lun=(hd->cmd[1]&0x60)>>5;
		if (hd->result==0) {
			return 0;
		} else {
			SCSI_LOG_DEBUG("SCSI HDFL: lun %d write error; check condition\n", lun);
			return (lun<<5)|2; // check condition with lun
		}
	}
	else {
		SCSI_LOG_DEBUG("SCSI HDFL: unimplemented status 0x%x\n", hd->cmd[0]);
		exit(1);
	}
	return 0; //ok
}

scsi_dev_t *scsi_dev_hd_new(const char *imagename) {
	scsi_hd_t *hd=calloc(sizeof(scsi_hd_t), 1);
	hd->writes_enabled=1;
	hd->hdfile=fopen(imagename, "r+b");
	if (!hd->hdfile) {
		perror(imagename);
		hd->writes_enabled=0;
		hd->hdfile=fopen(imagename, "rb");
		if (!hd->hdfile) {
			perror(imagename);
			free(hd);
			return NULL;
		}
	}
	hd->dev.handle_status=hd_handle_status;
	hd->dev.handle_cmd=hd_handle_cmd;
	hd->dev.handle_data_in=hd_handle_data_in;
	hd->dev.handle_data_out=hd_handle_data_out;
	return (scsi_dev_t*)hd;
}
