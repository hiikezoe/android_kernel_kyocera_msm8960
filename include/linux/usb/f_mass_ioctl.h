/*
 * Secure massstorage IOCTL I/F.
 *
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __F_MASS_IOCTL_H
#define __F_MASS_IOCTL_H


#include <linux/ioctl.h>

#define USBMAS_IOC_MAGIC      'M'

#define USBMAS_IOC_START_NOTIFY     _IO(USBMAS_IOC_MAGIC, 0)
#define USBMAS_IOC_GET_COMMAND      _IO(USBMAS_IOC_MAGIC, 1)
#define USBMAS_IOC_GET_FSGLUN       _IO(USBMAS_IOC_MAGIC, 2)
#define USBMAS_IOC_GET_DATA         _IO(USBMAS_IOC_MAGIC, 3)
#define USBMAS_IOC_TX_DATA          _IO(USBMAS_IOC_MAGIC, 4)
#define USBMAS_IOC_RX_DATA          _IO(USBMAS_IOC_MAGIC, 5)
#define USBMAS_IOC_DO_READ          _IO(USBMAS_IOC_MAGIC, 6)
#define USBMAS_IOC_DO_WRITE         _IO(USBMAS_IOC_MAGIC, 7)
#define USBMAS_IOC_NO_DATA          _IO(USBMAS_IOC_MAGIC, 8)
#define USBMAS_IOC_INVALID_COMMAND  _IO(USBMAS_IOC_MAGIC, 9)


/* Length of a SCSI Command Data Block */
#define USBMAS_MAX_COMMAND_SIZE 16

#define USBMAS_REQUEST_NONE       0
#define USBMAS_REQUEST_DATA       1
#define USBMAS_REQUEST_DATA_LAST  2
#define USBMAS_REQUEST_DATA_ERROR 3


/* start information */
struct ioc_startinfo_type
{
    uint8_t Data[12];
    int32_t DataSize;
    int32_t Offset;
};

/* command block wrapper */
struct ioc_bulk_cb_wrap_type {
    uint32_t TransferLength;
    uint8_t DataDir;
    uint8_t Lun;
    uint8_t CmdLength;
    uint8_t Cmd[USBMAS_MAX_COMMAND_SIZE];
};


/* fsg_lun */
struct ioc_fsg_lun_type {
    uint32_t Luns;
    uint32_t FileSts;
    uint32_t NumSectors;
    uint32_t Ro;
    uint32_t SenseData;
    uint32_t SenseDataInfo;
    uint32_t UnitAttentionData;
    uint32_t FsgBuffLen;
};


/* command atatus wrapper */
struct ioc_bulk_cs_wrap_type
{
    uint32_t Residue;       /* Amount not transferred */
    uint8_t Status;         /* See below */
    uint32_t SenseData;
};


/* data information */
struct ioc_data_type
{
    uint8_t *Buff;
    uint32_t BuffSize;
    int32_t  Request;
    int32_t  Status;
    struct ioc_bulk_cs_wrap_type  CSW;
};

union mass_ioctl_type
{
    struct ioc_startinfo_type     start_info;
    struct ioc_fsg_lun_type       lun_info;
    struct ioc_bulk_cb_wrap_type  cbw_info;
    struct ioc_bulk_cs_wrap_type  csw_info;
    struct ioc_data_type          data_info;
};

#endif /* __F_MASS_IOCTL_H */
