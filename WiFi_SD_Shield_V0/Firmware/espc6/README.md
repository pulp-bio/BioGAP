# Workflow

SD Card Initialization

| SPI Init | -> | mounting fat |

## SDCARD_SPI workflow
```mermaid
+----------------------+
| timer / SPI ISR      |
| (currently fake)     |
+----------------------+
           |
           | task notification
           v
+----------------------+
| read_sensor_task     |
|----------------------|
| wait notif           |
| fake/read SPI data   |
| prepare_buffer()     |
| enqueue ringbuffer   |
+----------------------+
           |
           v
+----------------------+
| global ring buffer   |
+----------------------+

```

## Speed Transfer (theoretical)

SPI rate, default value: 20 Mbit/s / 8 -> 2.5 MB/s
Time to transfer 16 KB -> 16 KB / 2.5 MB/s --> aprox 6.4 ms

BIOGAP 16 channels, 3 bytes channel -> 48 Bytes + other stuff -> 50 Bytes 
sampling at 1 Khz -> 50 bytes in 1 ms --> 500 bytes in 10 ms

So to fill 16 KB we need 32 transfers
32 * 10 ms -> 320 ms 



# Debugging

After flashing, originally encountered this error


I (323) [sd_card]: Initialize CARD SPI BUS
I (328) [sd_card]: MOSI=18 MISO=20 SCLK=19 CS=23
I (378) [sd_card]: SD CARD SPI successfully initialized
I (378) gpio: GPIO[23]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
I (424) sdspi_transaction: cmd=5, R1 response: command not supported
W (667) vfs_fat_sdmmc: failed to mount card (13)
E (668) vfs_fat_sdmmc: mount_to_vfs failed (0xffffffff).
I (668) gpio: GPIO[23]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0
E (677) [sd_card]: Failed to mount filesystem
E (682) [sd_card]: Failed to mount FAT filesystem: ESP_FAIL
I (688) main_task: Returned from app_main()

This is because the mount config was: 
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

Solved by: .format_if_mount_failed = true;
