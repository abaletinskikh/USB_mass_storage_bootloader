/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v2.0_Cube
  * @brief          : Memory management layer.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

struct msdos_volume_info {
    uint8_t drive_number;   /* BIOS drive number */
    uint8_t RESERVED;       /* Unused */
    uint8_t ext_boot_sign;  /* 0x29 if fields below exist (DOS 3.3+) */
    uint8_t volume_id[4];   /* Volume ID number */
    uint8_t volume_label[11];   /* Volume label */
    uint8_t fs_type[8];     /* Typically FAT12 or FAT16 */
} __attribute__ ((packed));

struct msdos_boot_sector {
    uint8_t boot_jump[3]; /* Boot strap short or near jump */
    uint8_t system_id[8]; /* Name - can be used to special case
           partition manager volumes */
    uint16_t sector_size; /* bytes per logical sector */
    uint8_t cluster_size; /* sectors/cluster */
    uint16_t reserved;    /* reserved sectors */
    uint8_t fats;   /* number of FATs */
    uint16_t dir_entries; /* root directory entries */
    uint16_t sectors;   /* number of sectors */
    uint8_t media;    /* media code (unused) */
    uint16_t fat_length;  /* sectors/FAT */
    uint16_t secs_track;  /* sectors per track */
    uint16_t heads;   /* number of heads */
    uint32_t hidden;    /* hidden sectors (unused) */
    uint32_t total_sect;  /* number of sectors (if sectors == 0) */
    struct msdos_volume_info vi;
    uint8_t boot_code[0];
} __attribute__ ((packed));

struct entry
{
  char  name[8];        // Short file name
  char  ext[3];         // Short file extension
  uint8_t attr;         // File Attributes
  uint8_t nt_reserved;
  uint8_t time_tenth;       // Create time, fine resolution
  uint8_t create_time[2];     // Create time
  uint8_t create_date[2];     // Create date
  uint8_t last_access_date[2];  // Last access date
  uint8_t first_cluster_hi[2];  // Start cluster num
  uint8_t write_time[2];      // Write time
  uint8_t write_date[2];      // Write date
  uint8_t first_cluster_lo[2];  // Start cluster num
  uint32_t size;          // Size of file
} __attribute__ ((packed));

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

#define STORAGE_LUN_NBR                  1
#define STORAGE_BLK_NBR                  (128 - SIZE_OF_BOOTLOADER)
#define STORAGE_BLK_SIZ                  0x400

/* USER CODE BEGIN PRIVATE_DEFINES */

#define STORAGE_SERVICE_SECTORS         3
// 0 - null boot sector
// 1 - sector with fat
// 2 - entry sector

#define STORAGE_BUSY_SECTORS            (STORAGE_BLK_NBR + STORAGE_SERVICE_SECTORS)
#define STORAGE_ALL_SECTORS             (STORAGE_BUSY_SECTORS + STORAGE_BLK_NBR)

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */
  
  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,	
  0x00,
  'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
}; 
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

const struct msdos_boot_sector templ =
{
    .boot_jump      =   { 0xeb, 0x3c, 0x90 },  //dummy_boot_jump
    .system_id      =   "mkfs.fat",
    .sector_size    =   1024,
    .cluster_size   =   1,
    .reserved       =   1,
    .fats           =   1,
    .dir_entries    =   32,
    .sectors        =   STORAGE_ALL_SECTORS,
    .media          =   0xf8,
    .fat_length     =   1,
    .secs_track     =   1,
    .heads          =   1,
    .hidden         =   0,
    .total_sect     =   0,
    .vi             =   {
    .drive_number   =   0x80,
    .ext_boot_sign  =   0x29,
    .volume_id      =   { 0x64, 0x43, 0xb3, 0x2a },
    .volume_label   =   "NO NAME    ",
    .fs_type        =   "FAT12   ",
    },
};

const char dummy_boot_code[] = "\x0e"    /* push cs */
    "\x1f"          /* pop ds */
    "\xbe\x5b\x7c"      /* mov si, offset message_txt */
    /* write_msg: */
    "\xac"          /* lodsb */
    "\x22\xc0"          /* and al, al */
    "\x74\x0b"          /* jz key_press */
    "\x56"          /* push si */
    "\xb4\x0e"          /* mov ah, 0eh */
    "\xbb\x07\x00"      /* mov bx, 0007h */
    "\xcd\x10"          /* int 10h */
    "\x5e"          /* pop si */
    "\xeb\xf0"          /* jmp write_msg */
    /* key_press: */
    "\x32\xe4"          /* xor ah, ah */
    "\xcd\x16"          /* int 16h */
    "\xcd\x19"          /* int 19h */
    "\xeb\xfe"          /* foo: jmp foo */
    /* message_txt: */
    "This is not a bootable disk.  Please insert a bootable floppy and\r\n"
"press any key to try again ... \r\n";

const struct entry firmware_file =
{
  .name         = "FIRMWARE",
  .ext        = "BIN",
  .attr         = 0x21,
  .create_time    = { 0x57, 0x7c },
  .create_date    = { 0x6a, 0x4d },
  .last_access_date   = { 0x6a, 0x4d },
  //.first_cluster_hi = { 0, 0 },
  .write_time     = { 0x57, 0x7c },
  .write_date     = { 0x6a, 0x4d },
  .first_cluster_lo = { 0x2, 0 },
  .size         = FLASH_PAGE_SIZE * STORAGE_BLK_NBR,
};

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

static void make_null_sector(uint8_t *);
static void mark_fat_cluster(uint8_t *, int32_t, uint32_t);
static void make_fat(uint8_t *);
static void make_entry(uint8_t *);

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes over USB FS IP
  * @param  lun:
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  .
  * @param  lun: .
  * @param  block_num: .
  * @param  block_size: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  *block_num  = STORAGE_ALL_SECTORS;
  *block_size = STORAGE_BLK_SIZ;
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */

  switch (blk_addr)
  {
    case 0:
      make_null_sector(buf);
      break;
    case 1:
      make_fat(buf);
      break;
    case 2:
      make_entry(buf);
      break;
    case STORAGE_SERVICE_SECTORS ... STORAGE_BUSY_SECTORS - 1:
      {
        const uint8_t *ptr = (const uint8_t *)(FLASH_BASE
                          + (blk_addr + SIZE_OF_BOOTLOADER - STORAGE_SERVICE_SECTORS) * FLASH_PAGE_SIZE);
        memcpy (buf, ptr, FLASH_PAGE_SIZE);
      }
      break;
    default:
      memset(buf, 0, FLASH_PAGE_SIZE);
  }

  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */
  FLASH_EraseInitTypeDef erase_config;
  uint32_t result, addr, len;

  if (blk_addr < STORAGE_BUSY_SECTORS)
    return (USBD_OK);
  addr = (FLASH_BASE + (blk_addr + SIZE_OF_BOOTLOADER - STORAGE_BUSY_SECTORS) * FLASH_PAGE_SIZE);
  len = FLASH_PAGE_SIZE / 8; //DOUBLE WORD WRITES
  HAL_FLASH_Unlock();
  FLASH_WaitForLastOperation(FLASH_TIMEOUT_VALUE);
  //Erase program space
  erase_config.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_config.Banks = FLASH_BANK_1;
  erase_config.PageAddress = addr;
  erase_config.NbPages = 1;
  HAL_FLASHEx_Erase(&erase_config, &result);
  //Programming
  while(len-- > 0)
  {
    uint64_t data;
    data = *(uint64_t *)buf;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                    addr, data);
    buf += 8;  //DOUBLE WORD 64 bit
    addr += 8;  //DOUBLE WORD 64 bit
  }
  HAL_FLASH_Lock();
  return (USBD_OK);
  /* USER CODE END 7 */
}

/**
  * @brief  .
  * @param  None
  * @retval .
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

static void make_null_sector(uint8_t *p)
{
  // Clear all data
  memset(p, 0, FLASH_PAGE_SIZE);
  memcpy(p, &templ, sizeof(templ));
  memcpy(&((struct msdos_boot_sector *)p)->boot_code,
      dummy_boot_code, sizeof(dummy_boot_code));
  // Marking
  ((uint16_t *)p)[0x1fe/2] = 0xaa55;
  return;
}

static void mark_fat_cluster(uint8_t *fat, int32_t cluster, uint32_t value)
{
  value &= 0x0fff;
  if (((cluster * 3) & 0x1) == 0) {
    fat[3 * cluster / 2] = (unsigned char)(value & 0x00ff);
    fat[(3 * cluster / 2) + 1] =
    (unsigned char)((fat[(3 * cluster / 2) + 1] & 0x00f0)
            | ((value & 0x0f00) >> 8));
  } else {
    fat[3 * cluster / 2] =
    (unsigned char)((fat[3 * cluster / 2] & 0x000f) |
            ((value & 0x000f) << 4));
    fat[(3 * cluster / 2) + 1] = (unsigned char)((value & 0x0ff0) >> 4);
  }
}

static void make_fat(uint8_t *fat_sector)
{
  uint16_t sector = STORAGE_SERVICE_SECTORS - 1;

  // Clear all data
  memset(fat_sector, 0, FLASH_PAGE_SIZE);
  // Set number of next sector
  while (sector < STORAGE_BUSY_SECTORS - 1)
  {
      mark_fat_cluster(fat_sector, sector, sector + 1);
      sector++;
  }
  //Begining fat
  mark_fat_cluster(fat_sector, 0, 0xff8);
  mark_fat_cluster(fat_sector, 1, 0xfff);
  // Set EOF file
  mark_fat_cluster(fat_sector, sector - 1, 0xfff);
  return;
}

static void make_entry(uint8_t *entry_sector)
{
  // Clear all data
  memset(entry_sector, 0, FLASH_PAGE_SIZE);
  // Make entry for our firmware file
  memcpy(entry_sector, &firmware_file, sizeof(firmware_file));
  return;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
