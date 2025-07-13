#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_mac.h"
#include "esp_spiffs.h"

#define TEMP_I2C_SDA 20
#define TEMP_I2C_SCL 21

#define RTC_I2C_SDA 33
#define RTC_I2C_SCL 34

#define TEMP_ADR 0x38

#define RTC_ADR 0x68

#define MAX_BYTES_WRITTEN 524288

#define Wait(x) vTaskDelay(x / portTICK_PERIOD_MS)

typedef struct
{
    i2c_master_dev_handle_t hDevice;
    float Temperature;
    float Humidity;
} TemperatureSensor;

typedef struct
{
    i2c_master_dev_handle_t hDevice;
    unsigned long long Epoch;
} RtcDevice;

uint32_t g_BytesWrittenToFile = 0;
i2c_master_bus_handle_t hBusTemperature;
i2c_master_bus_handle_t hBusRtc;

bool bInitTemperatureSensor(TemperatureSensor* Sensor, uint8_t Address)
{
    Wait(200);

    i2c_device_config_t DeviceConfig = {
        .device_address = Address,
        .scl_speed_hz = 100000
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(hBusTemperature, &DeviceConfig, &(Sensor->hDevice)));
    
    uint8_t Status = 0;
    Status = 0x71;

    ESP_ERROR_CHECK(i2c_master_transmit_receive(Sensor->hDevice, &Status, 1, &Status, 1, -1));

    return (Status & 0x18) == 0x18;
}

void vDestroyTemperatureSensor(TemperatureSensor* Sensor)
{
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(Sensor->hDevice));
}

bool bInitRtcDevice(RtcDevice* Rtc, uint8_t Address)
{
    i2c_device_config_t DeviceConfig = {
        .device_address = Address,
        .scl_speed_hz = 100000
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(hBusRtc, &DeviceConfig, &(Rtc->hDevice)));

    return true;
}

void vDestroyRtcDevice(RtcDevice* Rtc)
{
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(Rtc->hDevice));
}

void vInitI2CBus(uint8_t SCL, uint8_t SDA, i2c_master_bus_handle_t* hBus)
{
    i2c_master_bus_config_t I2CBusConfig = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = SCL,
        .sda_io_num = SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&I2CBusConfig, hBus));
}

void vResetI2CBus(i2c_master_bus_handle_t hBus)
{
    ESP_ERROR_CHECK(i2c_master_bus_reset(hBus));
}

void vDestroyI2CBus(i2c_master_bus_handle_t hBus)
{
    ESP_ERROR_CHECK(i2c_del_master_bus(hBus));
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

uint8_t BcdToBinary(uint8_t Bcd)
{
    return Bcd - 6 * (Bcd >> 4);
}

uint8_t BinaryToBcd(uint8_t Binary)
{
    return Binary + 6 * (Binary / 10);
}

void vReadRtcDevice(RtcDevice* Rtc)
{
    struct tm t;

    uint8_t buffer[8] = {0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00};
    
    ESP_ERROR_CHECK(i2c_master_transmit(Rtc->hDevice, buffer, 1, -1));
    
    ESP_ERROR_CHECK(i2c_master_receive(Rtc->hDevice, buffer, 7, -1));

    // uint8_t Years = BcdToBinary(buffer[6]) + 100;
    // // uint8_t Months = BcdToBinary(buffer[5]);
    // uint8_t Days = BcdToBinary(buffer[4]);
    // uint8_t Hours = BcdToBinary(buffer[2]);
    // uint8_t Minutes = BcdToBinary(buffer[1]);
    // uint8_t Seconds = BcdToBinary(buffer[0] & 0x7f);
    
    t.tm_year = 2000 + BcdToBinary(buffer[6]) - 1900;  // Full year minus 1900
    t.tm_mon  = BcdToBinary(buffer[5]) - 1;            // Months since January [0-11]
    t.tm_mday = BcdToBinary(buffer[4]);
    t.tm_hour = BcdToBinary(buffer[2]);
    t.tm_min  = BcdToBinary(buffer[1]);
    t.tm_sec  = BcdToBinary(buffer[0] & 0x7F);
    t.tm_isdst = 0;  // No daylight saving time

    Rtc->Epoch = mktime(&t);
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
    RtcDevice Rtc;

    vInitI2CBus(TEMP_I2C_SCL, TEMP_I2C_SDA, &hBusTemperature);

    bool TempInitStatus = bInitTemperatureSensor(&Sensor, TEMP_ADR);
    if (!TempInitStatus)
    {
        printf("e0t\n");
        vDestroyTemperatureSensor(&Sensor);
        fclose(LogFile);
        esp_vfs_spiffs_unregister(NULL);
        return;
    }
    else
    {
        printf("s0t\n");
    }

    vInitI2CBus(RTC_I2C_SCL, RTC_I2C_SDA, &hBusRtc);

    bool RtcInitStatus = bInitRtcDevice(&Rtc, RTC_ADR);
    if (!RtcInitStatus)
    {
        printf("e0r\n");
        vDestroyTemperatureSensor(&Sensor);
        vDestroyRtcDevice(&Rtc);
        fclose(LogFile);
        esp_vfs_spiffs_unregister(NULL);
        return;
    }
    else
    {
        printf("s0r\n");
    }

    vReadRtcDevice(&Rtc);

    if (iGetFileSize(LogFile) == 0)
    {
        while (1)
        {
            struct timeval Now;
            _gettimeofday_r(NULL, &Now, NULL);

            int64_t MicrosecondEpoch = (int64_t)Now.tv_sec * 1000000L + (int64_t)Now.tv_usec;
            int64_t MillisecondEpoch = MicrosecondEpoch / 1000L;

            vReadTemperatureSensor(&Sensor);

            unsigned char BufferToWrite[16];

            *((unsigned long long*)(BufferToWrite)) = MillisecondEpoch + Rtc.Epoch * 1000;
            *((float*)(BufferToWrite + 8)) = Sensor.Temperature;
            *((float*)(BufferToWrite + 12)) = Sensor.Humidity;

            fwrite(BufferToWrite, sizeof(BufferToWrite[0]), sizeof(BufferToWrite), LogFile);
            g_BytesWrittenToFile += sizeof(BufferToWrite);

            // printf("Time: %llu    Temp.: %.2f    Humidity: %.2f\n", MillisecondEpoch + Rtc.Epoch * 1000, Sensor.Temperature, Sensor.Humidity);

            // Quit
            if (getchar() == 'q' || g_BytesWrittenToFile > MAX_BYTES_WRITTEN) 
            {
                break;
            }
            // statUs
            else if (getchar() == 'u')
            {
                printf("s0\n");
            }
            // Index
            else if (getchar() == 'i')
            {
                printf("i%lu\n", g_BytesWrittenToFile / 16);
            }
        }
    }
    else
    {
        int FileSize = iGetFileSize(LogFile);
        int AmountOfEntries = FileSize / 16;
        
        while (1)
        {
            uint8_t Command[16];
            size_t CollectedChars = 0;
            bool BreakedEarly = false;
            while (CollectedChars < sizeof(Command) - 1)
            {
                uint8_t Char = getchar();
                if (Char !=  0xFFu)
                {
                    printf("%c", (char)Char);
                    Command[CollectedChars++] = Char;
                }
                if (Char == '\n')
                {
                    BreakedEarly = true;
                    break;
                }
                Wait(10);
            }
            Command[CollectedChars] = '\0';
            if (BreakedEarly)
            {
                Command[CollectedChars - 1] = '\0';
            } 
            // Quit
            if (Command[0] == 'q')
            {
                break;
            }
            // Read
            else if (Command[0] == 'r')
            {
                unsigned int EntryIndex = 0;
                if (sscanf((char*)Command + 1, "%u", &EntryIndex) == 1 && EntryIndex < AmountOfEntries)
                {
                    fseek(LogFile, EntryIndex * 16, SEEK_SET);
                    unsigned char Buffer[16];
                    fread(Buffer, sizeof(Buffer[0]), sizeof(Buffer), LogFile);

                    unsigned long long Timestamp = *((unsigned long long*)(Buffer));
                    float Temperature = *((float*)(Buffer + 8));
                    float Humidity = *((float*)(Buffer + 12));

                    printf("t%llup%.3fh%.3f\n", Timestamp, Temperature, Humidity);
                }
                else
                {
                    printf("e0e0e0\n");
                }
            }
            // Sample
            else if (Command[0] == 's')
            {
                vReadTemperatureSensor(&Sensor);
                struct timeval Now;
                _gettimeofday_r(NULL, &Now, NULL);

                int64_t MicrosecondEpoch = (int64_t)Now.tv_sec * 1000000L + (int64_t)Now.tv_usec;
                int64_t MillisecondEpoch = MicrosecondEpoch / 1000L;

                printf("t%llup%.3fh%.3f\n", MillisecondEpoch + Rtc.Epoch * 1000, Sensor.Temperature, Sensor.Humidity);
            }
            // Clear
            else if (Command[0] == 'c')
            {
                fclose(LogFile);
                LogFile = fopen("/spiffs/log.bin", "wb+");
                if (LogFile == NULL)
                {
                    printf("e0\n");
                }
                g_BytesWrittenToFile = 0;
                AmountOfEntries = 0;
                printf("s1\n");
            }
            // Index
            else if (Command[0] == 'i')
            {
                printf("i%u\n", AmountOfEntries);
            }
            // seT time
            else if (Command[0] == 't')
            {
                unsigned long long Epoch;
                if (sscanf((char*)Command + 1, "%llu", &Epoch) == 1)
                {
                    Epoch = Epoch / 1000;
                    struct tm t;
                    t.tm_year = (Epoch / 31536000) + 70;
                    t.tm_mon = ((Epoch % 31536000) / 2592000);
                    t.tm_mday = ((Epoch % 2592000) / 86400);
                    t.tm_hour = ((Epoch % 86400) / 3600);
                    t.tm_min = ((Epoch % 3600) / 60);
                    t.tm_sec = (Epoch % 60);
                    t.tm_isdst = -1;

                    Rtc.Epoch = Epoch;

                    uint8_t buffer[8];
                    buffer[0] = 0;
                    buffer[1] = BinaryToBcd(t.tm_sec);
                    buffer[2] = BinaryToBcd(t.tm_min);
                    buffer[3] = BinaryToBcd(t.tm_hour);
                    buffer[4] = 0;
                    buffer[5] = BinaryToBcd(t.tm_mday);
                    buffer[6] = BinaryToBcd(t.tm_mon + 1);
                    buffer[7] = BinaryToBcd(t.tm_year - 100);

                    ESP_ERROR_CHECK(i2c_master_transmit(Rtc.hDevice, buffer, sizeof(buffer), -1));
                    
                    printf("s1\n");
                }
                else
                {
                    printf("e0e0e0\n");
                }
            }
            // List
            else if (Command[0] == 'l')
            {
                fseek(LogFile, 0, SEEK_SET);
                unsigned char BufferToWrite[16];
                printf("i%u\n", AmountOfEntries);
                for (unsigned int i = 0; i < AmountOfEntries && i < 201; i++)
                {
                    fread(BufferToWrite, sizeof(BufferToWrite[0]), sizeof(BufferToWrite), LogFile);
                    unsigned long long Timestamp = *((unsigned long long*)(BufferToWrite));
                    float Temperature = *((float*)(BufferToWrite + 8));
                    float Humidity = *((float*)(BufferToWrite + 12));

                    printf("t%llup%.3fh%.3f\n", Timestamp, Temperature, Humidity);
                }
            }
            // statUs
            else if (Command[0] == 'u')
            {
                printf("a0\n");
            }
            else
            {
                printf("e-1e-1e-1\n");
            }
        }
    }

    vDestroyTemperatureSensor(&Sensor);
    vDestroyRtcDevice(&Rtc);
    vDestroyI2CBus(hBusRtc);
    vDestroyI2CBus(hBusTemperature);
    fclose(LogFile);
    esp_vfs_spiffs_unregister(NULL);
}