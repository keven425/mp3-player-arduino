/*
  AudioDevice.h - Library that wraps around one of two
  SeedStudio Grove MP3 players or the FM radio.
  Created by Cameron P. Bennett, March 29, 2016.
  Released into the public domain.
*/

#include "AudioDevice.h"

#define MP3_A_START_CODE        0x7E
#define MP3_A_END_CODE          0xEF
#define MP3_A_VERSION_CODE      0xFF
#define MP3_A_MAX_VOLUME        0x1E

#define MP3_B_START_CODE        0x7E
#define MP3_B_END_CODE          0x7E
#define MP3_B_MAX_VOLUME        0x1F

#define FM_MAX_VOLUME           0x0F
#define FM_MIN_SIGNAL_STRENGTH  30
#define FM_NUM_CONNECT_ATTEMPTS 5
#define FM_I2C_ADDR             0x10
#define FM_STATION_STEP         10

#define READ                    1
#define WRITE                   0


bool isPaused = true;

uint16_t gChipID = 0;
uint16_t frequency = 8850;
uint8_t RDA5807P_REGW[10];

uint8_t RDA5807N_initialization_reg[] = {
    0xC4, 0x01, //change 01 to 05 enables the RDS/RBDS
    0x00, 0x00,
    0x04, 0x00,
    0xC3, 0xad,  //05h
    0x60, 0x00,
    0x42, 0x12,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,  //0x0ah
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,
    0x00, 0x00,  //0x10h
    0x00, 0x19,
    0x2a, 0x11,
    0xB0, 0x42,
    0x2A, 0x11,  //
    0xb8, 0x31,  //0x15h
    0xc0, 0x00,
    0x2a, 0x91,
    0x94, 0x00,
    0x00, 0xa8,
    0xc4, 0x00,  //0x1ah
    0xF7, 0xcF,
    0x12, 0x14,  //0x1ch
    0x80, 0x6F,
    0x46, 0x08,
    0x00, 0x86,  //10000110
    0x06, 0x61,  //0x20H
    0x00, 0x00,
    0x10, 0x9E,
    0x23, 0xC8,
    0x04, 0x06,
    0x0E, 0x1C,  //0x25H     //0x04 0x08
};





AudioDevice::AudioDevice(int rx, int tx, Mode mode) {
    _mode = mode;
    if (mode == mp3a || mode == mp3b) {
        mp3 = new SoftwareSerial(rx, tx);
        mp3->begin(9600);
    }
}

void AudioDevice::initHardware() {
    switch (_mode) {
        case fm: {
            Wire.begin(); // Initializes the I2C hardware
            RDA5807P_Initialization(); // Soft resets the FM radio chip & loads the init registers
            RDA5807P_SetMute(true);
            RDA5807P_SetFreq(frequency);
            break;
        }
        case mp3a: {
            char data[] = {0x09, 0x00, 0x00, 0x02};
            mp3AWrite(6, &data[0], 200); // Set the music source to the SD card
            break;
        }
    }
}

void AudioDevice::setVolume(uint8_t vol) {
    uint8_t mappedVol;
    switch (_mode) {
        case fm: {
            mappedVol = map(vol, 0, 0xFF, 0, FM_MAX_VOLUME);
            RDA5807P_SetVolumeLevel(mappedVol);
            break;
        }
        case mp3a: {
            mappedVol = map(vol, 0, 0xFF, 0, MP3_A_MAX_VOLUME);
            char data[] = {0x06, 0x00, 0x00, mappedVol};
            mp3AWrite(6, &data[0], 10);
            break;
        }
        case mp3b: {
            mappedVol = map(vol, 0, 0xFF, 0, MP3_B_MAX_VOLUME);
            char data[] = {0xA7, mappedVol};
            mp3BWrite(3, &data[0], 10);
            break;
        }
    }
}

uint16_t AudioDevice::next() {
    uint16_t retVal = 0;
    switch (_mode) {
        case fm: {
            retVal = fmSeek(true);
            break;
        }
        case mp3a: {
            char data[] = {0x01, 0x00, 0x00, 0x00};
            mp3AWrite(6, &data[0], 10);
            break;
        }
        case mp3b: {
            char data[] = {0xA5};
            mp3BWrite(2, &data[0], 10);
            break;
        }
    }
    return retVal;
}

uint16_t AudioDevice::previous() {
    uint16_t retVal = 0;
    switch (_mode) {
        case fm: {
            retVal = fmSeek(false);
            break;
        }
        case mp3a: {
            char data[] = {0x02, 0x00, 0x00, 0x00};
            mp3AWrite(6, &data[0], 10);
            break;
        }
        case mp3b: {
            char data[] = {0xA6};
            mp3BWrite(2, &data[0], 10);
            break;
        }
    }
    return retVal;
}

void AudioDevice::setTrack(uint16_t track) {
    switch (_mode) {
        case fm: {
            RDA5807P_SetFreq(track);
            break;
        }
        case mp3a: {
            char data[] = {0x03, 0x00, track / 0xFF, track % 0xFF};
            mp3AWrite(6, &data[0], 10);
            break;
        }
        case mp3b: {
            char data[] = {0xA0, track / 0xFF, track % 0xFF};
            mp3BWrite(4, &data[0], 10);
            break;
        }
    }
}

void AudioDevice::play() {
    switch (_mode) {
        case fm: {
            RDA5807P_SetMute(false);
            break;
        }
        case mp3a: {
            char data[] = {0x0D, 0x00, 0x00, 0x00};
            mp3AWrite(6, &data[0], 10);
            break;
        }
        case mp3b: {
            char data[] = {0xA3};
            if (!isPaused) mp3BWrite(2, &data[0], 10); // command A3 toggles play/pause
            break;
        }
    }
}

void AudioDevice::pause() {
    switch (_mode) {
        case fm: {
            RDA5807P_SetMute(true);
            break;
        }
        case mp3a: {
            char data[] = {0x0E, 0x00, 0x00, 0x00};
            mp3AWrite(6, &data[0], 10);
            break;
        }
        case mp3b: {
            char data[] = {0xA3};
            if (isPaused) mp3BWrite(2, &data[0], 10); // command A3 toggles play/pause
            break;
        }
    }
}



void AudioDevice::mp3BWrite(uint8_t commandLength, char* data, int delayLength) {
    mp3->write(MP3_B_START_CODE);
    // commandLength includes the length commands, hence commandLength - 1
    for (int i = 0; i < commandLength - 1; i++) mp3->write(*(data + i));
    mp3->write(MP3_B_END_CODE);
    if (delayLength > 0) delay(delayLength);
}

void AudioDevice::mp3AWrite(uint8_t commandLength, char* data, int delayLength) {
    mp3->write(MP3_A_START_CODE);
    mp3->write(MP3_A_VERSION_CODE);
    mp3->write(commandLength);
    // commandLength includes the length and version commands, hence commandLength - 2
    for (int i = 0; i < commandLength - 2; i++) mp3->write(*(data + i));
    mp3->write(MP3_A_END_CODE);
    if (delayLength > 0) delay(delayLength);
}

uint16_t AudioDevice::fmSeek(boolean seekUp) {
    int signalStrength;
    do {
        do {
            Serial.println(frequency);
            frequency += seekUp ? FM_STATION_STEP : -FM_STATION_STEP;
            if (frequency > 10800) frequency = 8800;
            if (frequency < 8800) frequency = 10800;
        } while (!RDA5807P_ValidStop(frequency));
        Serial.println("Valid Stop");
        delay(50);
        signalStrength = RDA5807P_GetSigLvl();// max is 63 according to Data sheet, but I've seen more
        Serial.println(signalStrength);
    } while (signalStrength < FM_MIN_SIGNAL_STRENGTH);// minimum signal strength, keep looking
    Serial.println("Strong Signal");
    return frequency;
}

void AudioDevice::RDA5807P_SetFreq(int16_t frequency) {
    uint16_t curChan;
    curChan = RDA5807P_FreqToChan(frequency);

    if ((frequency >= 6500) && (frequency < 7600)) {
        RDA5807P_REGW[3] = 0x0c;
    } else if ((frequency >= 7600) && (frequency < 10800)) {
        RDA5807P_REGW[3] = 0x08;// sets the BAND bits (00xx = 87-108, 01xx=76-91, 10xx=76-108, 11xx=65-76
        // for north america this must be set to 10xx for some unknown reason
    }

    RDA5807P_REGW[0] |= 1 << 6;
    RDA5807P_REGW[2] = curChan >> 2;
    RDA5807P_REGW[3] = (((curChan & 0x0003) << 6) | 0x10) | (RDA5807P_REGW[3] & 0x0f); //set tune bit

    OperationRDAFM_2w(WRITE, &(RDA5807P_REGW[0]), 4);
    delay(50);
}

bool AudioDevice::RDA5807P_ValidStop(int freq) {
    uint8_t RDA5807P_reg_data[4] = { 0x00 };
    uint8_t falseStation = 0;

    if (freq >= 6500 && freq < 7600) {
        RDA5807P_REGW[3] = 0x0c;
    } else if (freq >= 7600 && freq < 10800) {
        RDA5807P_REGW[3] = 0x08;// sets the BAND bits (00xx = 87-108, 01xx=76-91, 10xx=76-108, 11xx=65-76
        // for north america this must be set to 10xx for some unknown reason
    }

    uint16_t curChannel = RDA5807P_FreqToChan(freq);

    RDA5807P_REGW[0] |= 1 << 6; // reg zero is bits 15 to bit 8 (this shifts to bit 14)
    RDA5807P_reg_data[0] = RDA5807P_REGW[0];
    RDA5807P_reg_data[1] = RDA5807P_REGW[1];
    RDA5807P_reg_data[2] = curChannel >> 2;//03H 15:8 CHAN
    RDA5807P_reg_data[3] = (((curChannel & 0x0003) << 6) | 0x10) | (RDA5807P_REGW[3] & 0x0f);//

    OperationRDAFM_2w(WRITE, &(RDA5807P_reg_data[0]), 4);

    delay(50);

    if (0x5808 == gChipID) {
        OperationRDAFM_2w(READ, &(RDA5807P_reg_data[0]), 4); //
    } else {
        int i = 0;
        do {
            if (++i > FM_NUM_CONNECT_ATTEMPTS) return false;
            delay(30);
            OperationRDAFM_2w(READ, &(RDA5807P_reg_data[0]), 4);
        } while ((RDA5807P_reg_data[0] & 0x40) == 0);
    }

    //check FM_TRUE
    if ((RDA5807P_reg_data[2] & 0x01) == 0) return false;
    if (freq == 9600) return false; // North America - if scanning DOWN, the radio will lock on 9600 for some reason!
    delay(50);
    return true;
}

uint8_t AudioDevice::RDA5807P_GetSigLvl() {
    uint8_t RDA5807P_reg_data[4] = { 0x00 };
    OperationRDAFM_2w(READ, &(RDA5807P_reg_data[0]), 4);
    delay(50);
    return (RDA5807P_reg_data[2] >> 1);
}

uint16_t AudioDevice::RDA5807P_FreqToChan(uint16_t frequency) {
    return (frequency - 7600) / 10;
}

void AudioDevice::RDA5807P_Initialization() {
    uint8_t RDA5807P_REGR[10] = { 0x0 };

    RDA5807P_REGW[0] = 0x00;
    RDA5807P_REGW[1] = 0x02;

    OperationRDAFM_2w(WRITE, (uint8_t *)&RDA5807P_REGW[0], 2); // soft reset
    delay(50);

    OperationRDAFM_2w(READ, (uint8_t *)&RDA5807P_REGR[0], 10);
    delay(50);

    gChipID = RDA5807P_REGR[8];
    gChipID = ((gChipID << 8) | RDA5807P_REGR[9]);

    for (int i = 0; i < 8; i++) RDA5807P_REGW[i] = RDA5807N_initialization_reg[i];

    OperationRDAFM_2w(WRITE, (uint8_t *)&RDA5807N_initialization_reg[0], 2); // power up
    delay(600);
    OperationRDAFM_2w(WRITE, (uint8_t *)&RDA5807N_initialization_reg[0], sizeof(RDA5807N_initialization_reg));
    delay(50);
}

void AudioDevice::RDA5807P_SetMute(boolean mute) {
    if (mute)
        RDA5807P_REGW[0] &=  ~(1 << 6);
    else
        RDA5807P_REGW[0] |= 1 << 6;

    OperationRDAFM_2w(WRITE, &(RDA5807P_REGW[0]), 2);
    delay(50);
}

void AudioDevice::RDA5807P_SetVolumeLevel(uint8_t level) {
    uint8_t RDA5807P_reg_data[8];
    for (int i = 0; i < 8; i++) RDA5807P_reg_data[i] = RDA5807P_REGW[i];

    RDA5807P_reg_data[7] = ((RDA5807P_REGW[7] & 0xF0) | (level & 0x0F));
    RDA5807P_reg_data[3] &= ~0x10; // disable tune

    OperationRDAFM_2w(WRITE, &(RDA5807P_reg_data[0]), 8);
    delay(50);
}

void AudioDevice::OperationRDAFM_2w(unsigned char operation, unsigned char *data, int numBytes) {
    if (operation == READ) {
        Wire.requestFrom(FM_I2C_ADDR, numBytes);
        for (int i = 0; i < numBytes; i++) *data++ = Wire.read();
    } else {
        Wire.beginTransmission(FM_I2C_ADDR);
        for (int i = 0; i < numBytes; i++) Wire.write(*data++);
        Wire.endTransmission();
    }
}
