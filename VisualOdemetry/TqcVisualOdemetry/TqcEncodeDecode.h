#ifndef TQCENCODEDECODE_H
#define TQCENCODEDECODE_H

/**
 * 实现了YUV像素数据编码为视频码流（H264，MPEG2，VP8等等）。
 * This software encode YUV420P data to H.264 bitstream.
 */

#include <stdio.h>
#include <opencv2/core/core.hpp>

using namespace cv;

#define __STDC_CONSTANT_MACROS

#ifdef _WIN32
// Windows
extern "C"
{
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
#include "libswscale/swscale.h"
};
#else
// Linux...
#ifdef __cplusplus
extern "C"
{
#endif
#include <libavutil/opt.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include "libavutil/avutil.h"
#include "libswscale/swscale.h"
#ifdef __cplusplus
};
#endif
#endif

class CEncodeDecode
{
#define TQC_FILE_NAME_LEN  256
#define TQC_INPUT_BUF_SIZE 4096

public:
    CEncodeDecode();
    ~CEncodeDecode();

    bool EncodeInit(const char *output, int nWidth, int nHeight);
    bool EncodeRelease();
    bool EncodeFrame(Mat &input);

    bool DecodeInit(const char *inputFile);
    bool DecodeRelease();
    bool Decode(Mat &yuv, int &nGotPic, bool bDecode = true);
    void ConvertYUV2BGR(AVCodecContext *pCodecCtx, SwsContext *img_convert_ctx, AVFrame *pFrame, Mat *pCvMat);
    int  GetTotalFrameNum();

private:
    int  CalcVideoFileTotalFrameNum();

private:
    char m_strOutputFile[TQC_FILE_NAME_LEN];
    char m_strInputFile[TQC_FILE_NAME_LEN];
    FILE *m_pOutputFile;
    FILE *m_pInputFile;

    // Encoder/Decoder
    AVFormatContext      *m_pFormatCtx;
    AVOutputFormat       *m_pOutputFmt;
    AVStream             *m_pVideoStream;
    AVCodecContext       *m_pCodecCtx;
    AVDictionary         *m_pParam;
    AVCodec              *m_pCodec;
    AVFrame              *m_pFrame;
    int                  m_nPicSize;
    uint8_t              *m_pPicBuf;
    AVPacket             m_packet;
    int                  m_nYSize;
    int                  m_nFrameCnt;
    int                  m_nTotalFrame;
    AVCodecParserContext *m_pCodecParserCtx;
    SwsContext           *m_ImgConvertCtx;

    uint8_t m_buf[TQC_INPUT_BUF_SIZE + AV_INPUT_BUFFER_PADDING_SIZE];
    uint8_t *m_pDataPtr;
    size_t  m_nDataSize;
    int     m_nFileSize;
    int     m_nReadFileSize;
};
#endif // TQCENCODEDECODE_H