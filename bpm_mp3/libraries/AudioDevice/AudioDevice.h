/*
  AudioDevice.h - Library that wraps around one of two
  SeedStudio Grove MP3 players or the FM radio.
  Created by Cameron P. Bennett, March 29, 2016.
  Minot modification by Alex Olwal, April 9, 2017. 
  Released into the public domain.
*/
#ifndef AudioDevice_h
#define AudioDevice_h

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

enum Mode { fm, mp3a, mp3b };

class AudioDevice
{
public:
    /**
     * Method: AudioDevice()
     * -----------------
     * Constructor for the AudioDevice class. If the music player is either one
     * of the mp3 players, pass the rx and tx pins as arguments. If the 
     * music player is the fm radio, the rx and tx arguments are ignored.
     */
    AudioDevice(int rx, int tx, Mode mode);

    /**
     * Method: initHardware()
     * -----------------
     * This must be called before playing any audio from the music players.
     */
    void initHardware(void);

    /**
     * Method: setVolume(uint8_t vol)
     * -----------------
     * Volume should be between 0 and 255. The volume argument is mapped to
     * the appropriate range for the music player used.
     */
    void setVolume(uint8_t vol);

    /**
     * Method: next()
     * -----------------
     * Plays the next track. If using the fm radio, seeks up until a radio
     * station with a good signal is found. The return value is 0 if using 
     * either mp3 player and the frequency of the new station otherwise. If
     * using either mp3 player, next() will start from the beginning of the
     * SD card if the last song is playing
     */
    uint16_t next(void);

    /**
     * Method: previous()
     * -----------------
     * Plays the previous track. If using the fm radio, seeks down until a radio
     * station with a good signal is found. The return value is 0 if using 
     * either mp3 player and the frequency of the new station otherwise. If
     * using either mp3 player, previous() will start from the end of the
     * SD card if the first song is playing
     */
    uint16_t previous(void);

    /**
     * Method: setTrack(uint16_t track)
     * -----------------
     * Plays the specified track. If using the fm radio, the provided 
     * argument should be a radio frequency, ideally between 8800 and 
     * 10800. For example, KQED (88.5) would be 8850. Otherwise, the 
     * argument is the index (beginning with 1) of a song on the SD card.
     * Behavior is undefined for indexes out of bounds.
     */
    void setTrack(uint16_t track);

    /**
     * Method: pause()
     * -----------------
     * Pauses the currently playing track. If the music is already paused,
     * pause() has no effect. If using the fm radio, mutes the radio.
     */
    void pause(void);

    /**
     * Method: play()
     * -----------------
     * Plays the currently paused track. If the music is already playing,
     * play() has no effect. If using the fm radio, un-mutes the radio.
     */
    void play(void);

    

private:
    Mode _mode;
    SoftwareSerial* mp3;

    void mp3BWrite(uint8_t commandLength, char* data, int delayLength);
    void mp3AWrite(uint8_t commandLength, char* data, int delayLength);

    uint16_t fmSeek(boolean seekUp);
    void RDA5807P_SetFreq(int16_t frequency);
    bool RDA5807P_ValidStop(int freq);
    uint8_t RDA5807P_GetSigLvl(void);
    uint16_t RDA5807P_FreqToChan(uint16_t frequency);
    void RDA5807P_Initialization(void);
    void RDA5807P_SetMute(bool mute);
    void RDA5807P_SetVolumeLevel(uint8_t level);
    void OperationRDAFM_2w(unsigned char operation, unsigned char *data, int numBytes);
};

#endif
