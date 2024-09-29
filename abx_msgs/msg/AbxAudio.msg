int32 AUDIO_FORMAT_PCM = 0
int32 AUDIO_FORMAT_G711alaw = 1
int32 AUDIO_FORMAT_G711ulaw = 2
int32 AUDIO_FORMAT_G726 = 3
int32 AUDIO_FORMAT_AAC = 4

int32 SAMPLE_RATE_8000 = 8000
int32 SAMPLE_RATE_16000 = 16000

int32 SAMPLE_DEPTH_16 = 16

time stamp
uint32 seq

int32 audioFormat
int32 sampleRate
int32 sampleDepth
int32 bitRate
int32 channels
uint32 size
uint8[] data

int32[] extInts
string[] extStrs
