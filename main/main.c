#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_mac.h"
#include "esp_spiffs.h"

#define TEMP_SDA 20
#define TEMP_SCL 21
#define TEMP_ADR 0x38

#define MAX_BYTES_WRITTEN 524288

#define Wait(x) vTaskDelay(x / portTICK_PERIOD_MS)

typedef struct
{
    i2c_master_dev_handle_t hDevice;
    float Temperature;
    float Humidity;
} TemperatureSensor;

uint32_t g_BytesWrittenToFile = 0;

bool bInitTemperatureSensor(TemperatureSensor* Sensor, uint8_t SCL, uint8_t SDA, uint8_t Address)
{
    Wait(200);

    i2c_master_bus_config_t I2CBusConfig = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = SCL,
        .sda_io_num = SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    i2c_master_bus_handle_t hBus;

    ESP_ERROR_CHECK(i2c_new_master_bus(&I2CBusConfig, &hBus));

    i2c_device_config_t DeviceConfig = {
        .device_address = Address,
        .scl_speed_hz = 100000
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(hBus, &DeviceConfig, &(Sensor->hDevice)));
    
    uint8_t Status = 0;
    Status = 0x71;

    ESP_ERROR_CHECK(i2c_master_transmit_receive(Sensor->hDevice, &Status, 1, &Status, 1, -1));

    return (Status & 0x18) == 0x18;
}

void vReadTemperatureSensor(TemperatureSensor* Sensor)
{
    uint8_t buffer[8] = {0xAC, 0x33, 0x00};

    ESP_ERROR_CHECK(i2c_master_transmit(Sensor->hDevice, buffer, 3, -1));

    Wait(100);

    bool HasSentFirstTransmission = false;
    while ((buffer[0] & 0b01000000) != 0 || !HasSentFirstTransmission)
    {
        buffer[0] = 0xAC;
        buffer[1] = 0x33;
        buffer[2] = 0x00;
        ESP_ERROR_CHECK(i2c_master_transmit_receive(Sensor->hDevice, buffer, 1, buffer, 1, -1));
        HasSentFirstTransmission = true;
    }

    Wait(10);

    ESP_ERROR_CHECK(i2c_master_receive(Sensor->hDevice, buffer, 7, -1));

    //             STATE       HUMIDITY              TEMPERATURE           CRC
    // buffer = 0bSSSSSSSS HHHHHHHH HHHHHHHH HHHHTTTT TTTTTTTT TTTTTTTT CCCCCCCC
    // discarding the state of the sensor, useless

    uint32_t RawHumidity = ((uint32_t)buffer[1] << 24) |
                      ((uint32_t)buffer[2] << 16) |
                      ((uint32_t)buffer[3] << 8)  |
                      ((uint32_t)buffer[4]);
    RawHumidity = (RawHumidity & 0xFFFFF000) >> 12;

    uint32_t RawTemperature = ((uint32_t)buffer[3] << 24) |
                      ((uint32_t)buffer[4] << 16) |
                      ((uint32_t)buffer[5] << 8)  |
                      ((uint32_t)buffer[6]);
    RawTemperature = (RawTemperature & 0x0FFFFF00) >> 8;

    Sensor->Humidity = (RawHumidity / (float)(1 << 20)) * 100.f;
    Sensor->Temperature = ((RawTemperature / (float)(1 << 20)) * 200.f) - 50.f;
}

long int iGetFileSize(FILE* File)
{
    long int CurrentPos = ftell(File);
    fseek(File, 0, SEEK_END);
    long int FileSize = ftell(File);
    fseek(File, CurrentPos, SEEK_SET);
    return FileSize;
}

void app_main(void)
{
    esp_vfs_spiffs_conf_t SpiffsConfig = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 1,
        .format_if_mount_failed = true
    };

    esp_vfs_spiffs_register(&SpiffsConfig);

    FILE* LogFile = fopen("/spiffs/log.bin", "rb+");

    TemperatureSensor Sensor;
    bool InitStatus = bInitTemperatureSensor(&Sensor, TEMP_SCL, TEMP_SDA, TEMP_ADR);
    printf("Init status: %d\n", InitStatus);
    if (!InitStatus)
    {
        printf("Failed to initialize sensor!\n");
        fclose(LogFile);
        esp_vfs_spiffs_unregister(NULL);
        return;
    }
    if (iGetFileSize(LogFile) == 0)
    {
        printf("Sampling\n");
        while (1)
        {
            struct timeval Now;
            _gettimeofday_r(NULL, &Now, NULL);

            int64_t MicrosecondEpoch = (int64_t)Now.tv_sec * 1000000L + (int64_t)Now.tv_usec;
            int64_t MillisecondEpoch = MicrosecondEpoch / 1000L;

            vReadTemperatureSensor(&Sensor);

            unsigned char BufferToWrite[16];

            *((unsigned long long*)(BufferToWrite)) = MillisecondEpoch;
            *((float*)(BufferToWrite + 8)) = Sensor.Temperature;
            *((float*)(BufferToWrite + 12)) = Sensor.Humidity;

            fwrite(BufferToWrite, sizeof(BufferToWrite[0]), sizeof(BufferToWrite), LogFile);
            g_BytesWrittenToFile += sizeof(BufferToWrite);

            printf("Time: %llu    Temp.: %.2f    Humidity: %.2f\n", MillisecondEpoch, Sensor.Temperature, Sensor.Humidity);

            if (getchar() == 'q' || g_BytesWrittenToFile > MAX_BYTES_WRITTEN) 
            {
                break;
            }
        }
    }
    else
    {
        long int FileSize = iGetFileSize(LogFile);
        long int AmountOfEntries = FileSize / 16;
        printf("File size: %ld bytes, Amount of entries: %ld\n", FileSize, AmountOfEntries);
        printf("Enter command (q to quit, r to read):\n> ");
        
        while (1)
        {
            char Command[16];
            size_t CollectedChars = 0;
            bool BreakedEarly = false;
            while (CollectedChars < sizeof(Command) - 1)
            {
                int c = getchar();
                if (c != EOF)
                {
                    Command[CollectedChars++] = (char)c;
                }
                if (c == '\n')
                {
                    BreakedEarly = true;
                    break;
                }
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
            Command[CollectedChars] = '\0';
            if (BreakedEarly)
            {
                Command[CollectedChars - 1] = '\0';
            }
            printf("%s\n", Command);
            if (Command[0] == 'q')
            {
                break;
            }
            else if (Command[0] == 'r')
            {
                unsigned int EntryIndex = 0;
                if (sscanf(Command, "r %d", &EntryIndex) == 1)
                {
                    EntryIndex = EntryIndex - 1;
                    unsigned char BufferToRead[16];
                    fseek(LogFile, EntryIndex * sizeof(BufferToRead), SEEK_SET);
                    size_t ReadBytes = fread(BufferToRead, sizeof(BufferToRead[0]), sizeof(BufferToRead), LogFile);
                    if (ReadBytes != sizeof(BufferToRead))
                    {
                        printf("Failed to read entry %d\n> ", ++EntryIndex);
                        continue;
                    }
                    int64_t MillisecondEpoch = *((int64_t*)(BufferToRead));
                    float Temperature = *((float*)(BufferToRead + 8));
                    float Humidity = *((float*)(BufferToRead + 12));
                    printf("Entry %d: Time: %lld    Temp.: %.2f    Humidity: %.2f\n", 
                            (int)++EntryIndex, MillisecondEpoch, Temperature, Humidity);
                }
                else
                {
                    for (size_t CurrentIdx = 0; CurrentIdx < AmountOfEntries; CurrentIdx++)
                    {
                        unsigned char BufferToRead[16];
                        fseek(LogFile, CurrentIdx * sizeof(BufferToRead), SEEK_SET);
                        size_t ReadBytes = fread(BufferToRead, sizeof(BufferToRead[0]), sizeof(BufferToRead), LogFile);
                        if (ReadBytes != sizeof(BufferToRead))
                        {
                            printf("Failed to read entry %d\n", CurrentIdx);
                            break;
                        }
                        int64_t MillisecondEpoch = *((int64_t*)(BufferToRead));
                        float Temperature = *((float*)(BufferToRead + 8));
                        float Humidity = *((float*)(BufferToRead + 12));
                        printf("Entry %d: Time: %lld    Temp.: %.2f    Humidity: %.2f\n", 
                               (int)CurrentIdx, MillisecondEpoch, Temperature, Humidity);
                    }
                }
                printf("> ");
            }
            else if (Command[0] == 'c')
            {
                fclose(LogFile);
                LogFile = fopen("/spiffs/log.bin", "wb+");
                if (LogFile == NULL)
                {
                    printf("Failed to clear log file!\n");
                    break;
                }
                g_BytesWrittenToFile = 0;
                printf("Log file cleared.\n");
                break;
            }
            else
            {
                printf("Unknown command: %s\n", Command);
            }
        }
    }

    fclose(LogFile);
    esp_vfs_spiffs_unregister(NULL);
}