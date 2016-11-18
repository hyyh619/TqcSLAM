/**
 * 实现了YUV像素数据编码为视频码流（H264，MPEG2，VP8等等）。
 * This software encode YUV420P data to H.264 bitstream.
 */

#include <opencv2/core/core.hpp>
#include <QDebug>
#include "TqcEncodeDecode.h"

int FlushEncoder(AVFormatContext *m_pOutputFmt_ctx, unsigned int stream_index)
{
    int      ret;
    int      got_frame;
    AVPacket enc_m_packet;

    if (!(m_pOutputFmt_ctx->streams[stream_index]->codec->codec->capabilities & CODEC_CAP_DELAY))
        return 0;

    while (1)
    {
        enc_m_packet.data = NULL;
        enc_m_packet.size = 0;
        av_init_packet(&enc_m_packet);
        ret = avcodec_encode_video2 (m_pOutputFmt_ctx->streams[stream_index]->codec, &enc_m_packet,
                                     NULL, &got_frame);
        av_frame_free(NULL);
        if (ret < 0)
            break;

        if (!got_frame)
        {
            ret = 0;
            break;
        }

        printf("Flush Encoder: Succeed to encode 1 frame!\tsize:%5d\n", enc_m_packet.size);
        /* mux encoded frame */
        ret = av_write_frame(m_pOutputFmt_ctx, &enc_m_packet);
        if (ret < 0)
            break;
    }

    return ret;
}

CEncodeDecode::CEncodeDecode()
{
    memset(m_strOutputFile, 0, TQC_FILE_NAME_LEN);
    memset(m_strInputFile, 0, TQC_FILE_NAME_LEN);

    m_pOutputFile     = NULL;
    m_pFormatCtx      = NULL;
    m_pOutputFmt      = NULL;
    m_pVideoStream    = NULL;
    m_pCodecCtx       = NULL;
    m_pParam          = NULL;
    m_pCodec          = NULL;
    m_pFrame          = NULL;
    m_nPicSize        = 0;
    m_pPicBuf         = NULL;
    m_nYSize          = 0;
    m_nFrameCnt       = 0;
    m_pCodecParserCtx = NULL;
    m_pInputFile      = NULL;
    m_pDataPtr        = NULL;
    m_nDataSize       = 0;
    m_ImgConvertCtx   = NULL;
    m_nFileSize       = 0;
    m_nReadFileSize   = 0;
}

CEncodeDecode::~CEncodeDecode()
{}

bool CEncodeDecode::EncodeInit(const char *output, int nWidth, int nHeight)
{
    size_t nLen = strlen(output);

    memcpy(m_strOutputFile, output, nLen);

    // const char* out_file = "src01.h264";
    // const char* out_file = "src01.ts";
    // const char* out_file = "src01.hevc";
    m_pOutputFile = fopen(output, "wb");
    if (!m_pOutputFile)
    {
        qDebug() << "Cannot open file:" << output;
        return false;
    }

    fclose(m_pOutputFile);

    av_register_all();

    // Method1.
    m_pFormatCtx = avformat_alloc_context();

    // Guess Format
    m_pOutputFmt          = av_guess_format(NULL, output, NULL);
    m_pFormatCtx->oformat = m_pOutputFmt;

    // Method 2.
    // avformat_alloc_output_context2(&m_pFormatCtx, NULL, NULL, out_file);
    // m_pOutputFmt = m_pFormatCtx->oformat;

    // Open output URL
    if (avio_open(&m_pFormatCtx->pb, output, AVIO_FLAG_READ_WRITE) < 0)
    {
        printf("Failed to open output file! \n");
        return -1;
    }

    m_pVideoStream = avformat_new_stream(m_pFormatCtx, 0);
    // m_pVideoStream->time_base.num = 1;
    // m_pVideoStream->time_base.den = 25;

    if (m_pVideoStream == NULL)
    {
        qDebug() << "Cannot get video stream frome format m_pCodecCtx.";
        return false;
    }

    // Param that must set
    m_pCodecCtx = m_pVideoStream->codec;

    // m_pCodecCtx->codec_id =AV_CODEC_ID_HEVC;
    m_pCodecCtx->codec_id      = m_pOutputFmt->video_codec;
    m_pCodecCtx->codec_type    = AVMEDIA_TYPE_VIDEO;
    m_pCodecCtx->pix_fmt       = AV_PIX_FMT_YUV420P;
    m_pCodecCtx->width         = nWidth;
    m_pCodecCtx->height        = nHeight;
    m_pCodecCtx->bit_rate      = 400000;
    m_pCodecCtx->gop_size      = 250;
    m_pCodecCtx->time_base.num = 1;
    m_pCodecCtx->time_base.den = 25;

    // H264
    // m_pCodecCtx->me_range = 16;
    // m_pCodecCtx->max_qdiff = 4;
    // m_pCodecCtx->qcompress = 0.6;
    m_pCodecCtx->qmin = 10;
    m_pCodecCtx->qmax = 51;

    // Optional Param
    m_pCodecCtx->max_b_frames = 3;

    // H.264
    if (m_pCodecCtx->codec_id == AV_CODEC_ID_H264)
    {
        av_dict_set(&m_pParam, "preset", "slow", 0);
        av_dict_set(&m_pParam, "tune", "zerolatency", 0);
        // av_dict_set(¶m, "profile", "main", 0);
    }

    // H.265
    if (m_pCodecCtx->codec_id == AV_CODEC_ID_H265)
    {
        av_dict_set(&m_pParam, "preset", "ultrafast", 0);
        av_dict_set(&m_pParam, "tune", "zero-latency", 0);
    }

    // Show some Information
    av_dump_format(m_pFormatCtx, 0, output, 1);

    m_pCodec = avcodec_find_encoder(m_pCodecCtx->codec_id);
    if (!m_pCodec)
    {
        qDebug() << "Can not find encoder! \n";
        EncodeRelease();
        return false;
    }

    if (avcodec_open2(m_pCodecCtx, m_pCodec, &m_pParam) < 0)
    {
        qDebug() << "Failed to open encoder! \n";
        EncodeRelease();
        return false;
    }

    m_pFrame   = av_frame_alloc();
    m_nPicSize = avpicture_get_size(m_pCodecCtx->pix_fmt, m_pCodecCtx->width, m_pCodecCtx->height);
    m_pPicBuf  = (uint8_t*)av_malloc(m_nPicSize);
    avpicture_fill((AVPicture*)m_pFrame, m_pPicBuf, m_pCodecCtx->pix_fmt, m_pCodecCtx->width, m_pCodecCtx->height);

    // Write File Header
    int err = avformat_write_header(m_pFormatCtx, NULL);
    if (err)
    {
        qDebug() << "avformat_write_header failure with err: " << err;
        EncodeRelease();
        return false;
    }

    av_new_packet(&m_packet, m_nPicSize);

    m_nYSize = m_pCodecCtx->width * m_pCodecCtx->height;

    return true;
}

bool CEncodeDecode::EncodeRelease()
{
    // Flush Encoder
    if (m_pFormatCtx)
    {
        int ret = FlushEncoder(m_pFormatCtx, 0);
        if (ret < 0)
        {
            qDebug() << ("Flushing encoder failed\n");
            return false;
        }
    }
    else
    {
        qDebug() << ("There is no format m_pCodecCtx.\n");
        return false;
    }

    // Write file trailer
    av_write_trailer(m_pFormatCtx);

    // Clean
    if (m_pVideoStream)
    {
        avcodec_close(m_pVideoStream->codec);

        if (m_pFrame)
            av_free(m_pFrame);

        if (m_pPicBuf)
            av_free(m_pPicBuf);
    }

    avio_close(m_pFormatCtx->pb);
    avformat_free_context(m_pFormatCtx);

    m_pFormatCtx = NULL;
    m_pFrame     = NULL;
    m_pPicBuf    = NULL;
    m_nFrameCnt  = 0;

    return true;
}

bool CEncodeDecode::EncodeFrame(Mat &input)
{
    int nGotPic = 0;

    memcpy(m_pPicBuf, input.data, m_nPicSize);

    m_pFrame->data[0] = m_pPicBuf;                      // Y
    m_pFrame->data[1] = m_pPicBuf + m_nYSize;           // U
    m_pFrame->data[2] = m_pPicBuf + m_nYSize * 5 / 4;   // V
                                                        // PTS
                                                        // m_pFrame->pts=i;
    m_pFrame->pts = m_nFrameCnt * (m_pVideoStream->time_base.den) / ((m_pVideoStream->time_base.num) * 25);

    // Encode
    int ret = avcodec_encode_video2(m_pCodecCtx, &m_packet, m_pFrame, &nGotPic);
    if (ret < 0)
    {
        qDebug() << ("Failed to encode! \n");
        return false;
    }

    if (nGotPic == 1)
    {
        qDebug() << "Succeed to encode frame: " << m_nFrameCnt;
        m_packet.stream_index = m_pVideoStream->index;
        ret                   = av_write_frame(m_pFormatCtx, &m_packet);
        av_free_packet(&m_packet);
    }

    m_nFrameCnt++;

    return true;
}

bool CEncodeDecode::DecodeInit(const char *inputFile)
{
    m_pInputFile = fopen(inputFile, "rb");
    if (!m_pInputFile)
    {
        qDebug() << "Cannot open file: " << inputFile;
        return false;
    }

    fseek(m_pInputFile, 0, SEEK_END);
    m_nFileSize = ftell(m_pInputFile);
    fseek(m_pInputFile, 0, SEEK_SET);

    avcodec_register_all();
    av_init_packet(&m_packet);

    m_pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!m_pCodec)
    {
        DecodeRelease();
        qDebug() << ("avcodec_find_encoder failed");
        return false;
    }

    m_pCodecCtx = avcodec_alloc_context3(m_pCodec);
    if (!m_pCodecCtx)
    {
        DecodeRelease();
        qDebug() << ("avcodec_alloc_context3 failed");
        return false;
    }

    m_pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!m_pCodecParserCtx)
    {
        qDebug() << "Could not allocate video parser context\n";
        return false;
    }

    m_pCodecCtx->codec_type = AVMEDIA_TYPE_VIDEO;
    m_pCodecCtx->pix_fmt    = AV_PIX_FMT_YUV420P;

    if (avcodec_open2(m_pCodecCtx, m_pCodec, NULL) < 0)
    {
        DecodeRelease();
        qDebug() << ("avcodec_open2 failed");
        return false;
    }

    m_pFrame = av_frame_alloc();
    if (!m_pFrame)
    {
        DecodeRelease();
        qDebug() << "av_frame_alloc failed";
        return false;
    }

    m_nFrameCnt = 0;
    CalcVideoFileTotalFrameNum();

    return true;
}

bool CEncodeDecode::DecodeRelease()
{
    // We open nothing. So, wo don't need to release something.
    if (m_pInputFile == NULL)
    {
        return true;
    }
    
    if (m_pCodecCtx)
    {
        avcodec_close(m_pCodecCtx);
        av_free(m_pCodecCtx);
        m_pCodecCtx = NULL;
    }

    if (m_pFrame)
    {
        av_frame_free(&m_pFrame);
        m_pFrame = NULL;
    }

    if (m_pInputFile)
    {
        fclose(m_pInputFile);
        m_pInputFile = NULL;
    }

    av_free_packet(&m_packet);

    if (m_pCodecParserCtx)
    {
        av_parser_close(m_pCodecParserCtx);
        m_pCodecParserCtx = NULL;
    }

    m_nFrameCnt     = 0;
    m_nDataSize     = 0;
    m_pDataPtr      = NULL;
    m_nFileSize     = 0;
    m_nReadFileSize = 0;

    return true;
}

bool CEncodeDecode::Decode(Mat &yuv, int &nGotPic, bool bDecode)
{
    int len;

    if (m_nDataSize == 0)
    {
        if ((m_nReadFileSize + TQC_INPUT_BUF_SIZE) > m_nFileSize)
            return false;

        m_nDataSize = fread(m_buf, 1, TQC_INPUT_BUF_SIZE, m_pInputFile);
        if (0 == m_nDataSize)
        {
            return false;
        }

        m_pDataPtr       = m_buf;
        m_nReadFileSize += m_nDataSize;
    }

    while (m_nDataSize > 0)
    {
        len = av_parser_parse2(m_pCodecParserCtx, m_pCodecCtx,
                               &(m_packet.data), &(m_packet.size),
                               m_pDataPtr, m_nDataSize,
                               AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        m_pDataPtr  += len;
        m_nDataSize -= len;

        if (0 == m_packet.size)
        {
            continue;
        }

        // qDebug() << "Parse 1 packet. Packet pts: " << m_packet.pts << endl;

        int ret = avcodec_decode_video2(m_pCodecCtx, m_pFrame, &nGotPic, &(m_packet));
        if (ret < 0)
        {
            qDebug() << "Decode Error.\n";
            return false;
        }

        if (nGotPic)
        {
            if (bDecode)
            {
                m_nFrameCnt++;
                qDebug() << "Succeed to decode frame: " << m_nFrameCnt;

                // 根据编码信息设置渲染格式
                if (m_ImgConvertCtx == NULL)
                {
                    m_ImgConvertCtx = sws_getContext(m_pCodecCtx->width, m_pCodecCtx->height,
                                                     m_pCodecCtx->pix_fmt, m_pCodecCtx->width, m_pCodecCtx->height,
                                                     AV_PIX_FMT_BGR24, SWS_BICUBIC, NULL, NULL, NULL);
                }

                // opencv
                if (yuv.empty())
                {
                    yuv.create(cv::Size(m_pCodecCtx->width, m_pCodecCtx->height), CV_8UC3);
                }

                if (m_ImgConvertCtx != NULL)
                {
                    ConvertYUV2BGR(m_pCodecCtx, m_ImgConvertCtx, m_pFrame, &yuv);
                }
            }

            return true;
        }
    }

    return true;
}

void CEncodeDecode::ConvertYUV2BGR(AVCodecContext *pCodecCtx, SwsContext *img_convert_ctx, AVFrame *pFrame, Mat *pCvMat)
{
    if (pCvMat->empty())
    {
        pCvMat->create(cv::Size(pCodecCtx->width, pCodecCtx->height), CV_8UC3);
    }

    AVFrame *pFrameRGB     = NULL;
    uint8_t *out_bufferRGB = NULL;

    pFrameRGB = av_frame_alloc();

    // 给pFrameRGB帧加上分配的内存;
    int size = avpicture_get_size(AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);
    out_bufferRGB = new uint8_t[size];
    avpicture_fill((AVPicture*)pFrameRGB, out_bufferRGB, AV_PIX_FMT_BGR24, pCodecCtx->width, pCodecCtx->height);

    // YUV to RGB
    sws_scale(img_convert_ctx, pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);

    memcpy(pCvMat->data, out_bufferRGB, size);

    delete[] out_bufferRGB;
    av_free(pFrameRGB);
}

int CEncodeDecode::GetTotalFrameNum()
{
    return m_nTotalFrame;
}

int CEncodeDecode::CalcVideoFileTotalFrameNum()
{
    bool res = false;
    Mat  frame;
    int  nGotPic = 0;

    m_nTotalFrame = 0;

    // We should go to the start.
    fseek(m_pInputFile, 0, SEEK_SET);

    do
    {
        res = Decode(frame, nGotPic, false);
        if (res && nGotPic)
        {
            m_nTotalFrame++;
        }

        nGotPic = 0;
    }
    while (res);

    // We will restart again.
    fseek(m_pInputFile, 0, SEEK_SET);

    m_nFrameCnt     = 0;
    m_nDataSize     = 0;
    m_pDataPtr      = NULL;
    m_nReadFileSize = 0;

    return m_nTotalFrame;
}
