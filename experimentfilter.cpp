#include "experimentfilter.h"

class ExperimentFilterRunnable;

// helper function
static int getCvType(QVideoFrame::PixelFormat pixelFormat) {
    switch(pixelFormat) {
    case QVideoFrame::Format_ARGB32:
        return CV_8UC4;
    case QVideoFrame::Format_ARGB32_Premultiplied:
        return CV_8UC4;
    case QVideoFrame::Format_RGB32:
        return CV_8UC4;
    case QVideoFrame::Format_RGB24:
        return CV_8UC3;
    case QVideoFrame::Format_RGB565:
        return CV_8UC2;
    case QVideoFrame::Format_RGB555:
        return CV_8UC2;
    case QVideoFrame::Format_ARGB8565_Premultiplied:
        return CV_8UC3;
    case QVideoFrame::Format_BGRA32:
        return CV_8UC4;
    case QVideoFrame::Format_BGRA32_Premultiplied:
        return CV_8UC4;
    case QVideoFrame::Format_BGR32:
        return CV_8UC4;
    case QVideoFrame::Format_BGR24:
        return CV_8UC3;
    case QVideoFrame::Format_BGR565:
        return CV_8UC2;
    case QVideoFrame::Format_BGR555:
        return CV_8UC2;
    case QVideoFrame::Format_BGRA5658_Premultiplied:
        return CV_8UC3;
    case QVideoFrame::Format_YUV420P:
        return CV_8UC1;
    case QVideoFrame::Format_NV12:
        return CV_8UC1;
    default:
        qCritical() << pixelFormat;
        qFatal("unknown pixel format");
    }
}

ExperimentFilterRunnable::~ExperimentFilterRunnable(){}

QVideoFrame ExperimentFilterRunnable::run(QVideoFrame *inputFrame, const QVideoSurfaceFormat &surfaceFormat, QVideoFilterRunnable::RunFlags flags){

    auto size(inputFrame->size());
    auto height(size.height());
    auto width(size.width());

    inputFrame->map(QAbstractVideoBuffer::ReadOnly);

    auto pixelFormat(inputFrame->pixelFormat());
    auto inputType(getCvType(pixelFormat));

    auto bits(inputFrame->bits());
    auto bytesPerLine(inputFrame->bytesPerLine());

    auto inputMat(cv::Mat(height, width, inputType, bits, bytesPerLine));

    // add salt and pepper to disturb confidence value of marker tracker
    if(*(noiseMagnitude) > 0){
        cv::Mat saltAndPepper = cv::Mat::zeros(height, width, inputType);
        cv::randu(saltAndPepper, LOWER_NOISE_THRESHHOLD, UPPER_NOISE_THRESHHOLD);

        cv::Mat black = saltAndPepper > UPPER_NOISE_THRESHHOLD - (*noiseMagnitude)*((UPPER_NOISE_THRESHHOLD-LOWER_NOISE_THRESHHOLD)/2);
        cv::Mat white = saltAndPepper < LOWER_NOISE_THRESHHOLD + (*noiseMagnitude)*((UPPER_NOISE_THRESHHOLD-LOWER_NOISE_THRESHHOLD)/2);

        inputMat.setTo(255, white);
        inputMat.setTo(0, black);
        }

    // blacking out some parts of the image
    cv::Mat blackOut = cv::Mat::zeros(height, width, inputType);

    auto halfHeight = height/2; auto halfWidth = width/2;

    cv::Rect recUpperLeft = cv::Rect(0, 0, halfWidth, halfHeight);
    cv::Rect recUpperRight = cv::Rect(halfWidth, 0, halfWidth, halfHeight);
    cv::Rect recLowerLeft = cv::Rect(0, halfHeight, halfWidth, halfHeight);
    cv::Rect recLowerRight = cv::Rect(halfWidth, halfHeight, halfWidth, halfHeight);

    if(hideFlags[0]) inputMat(recUpperLeft) = 0;
    if(hideFlags[1]) inputMat(recUpperRight) = 0;
    if(hideFlags[2]) inputMat(recLowerLeft) = 0;
    if(hideFlags[3]) inputMat(recLowerRight) = 0;

    inputFrame->unmap();

    return *inputFrame;

}


