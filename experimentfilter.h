#ifndef EXPERIMENTFILTER_H
#define EXPERIMENTFILTER_H

#include <QVideoFilterRunnable>

#include <opencv2/video.hpp>
#include <opencv2/core.hpp>
#include <QAbstractVideoFilter>

#include "vision-video-filter.h"

class ExperimentFilterRunnable : public QVideoFilterRunnable{

public:
    ExperimentFilterRunnable(float* noiseMagnitude, bool *hideFlags)
    : noiseMagnitude(noiseMagnitude)
    , hideFlags(hideFlags)
    {}

    ~ExperimentFilterRunnable();

    QVideoFrame run(QVideoFrame *inputFrame, const QVideoSurfaceFormat &surfaceFormat, RunFlags flags);

protected:
    // parameter for salt and pepper noise
    float* noiseMagnitude;
    const int UPPER_NOISE_THRESHHOLD = 255;
    const int LOWER_NOISE_THRESHHOLD = 0;

    // parameter for hiding
    bool *hideFlags;
};

class ExperimentFilter : public QAbstractVideoFilter {

    Q_OBJECT

public:
    Q_INVOKABLE void setNoiseMagnitude(const float &m){ noiseMagnitude = m; }
    Q_INVOKABLE void setHideFlag(const int &i, const bool b) { hideFlags[i] = b; }
    QVideoFilterRunnable* createFilterRunnable() { return new ExperimentFilterRunnable(&noiseMagnitude, hideFlags); }

protected:
    float noiseMagnitude = 0;
    bool hideFlags[4] = {false, false, false, false};
};



#endif // EXPERIMENTFILTER_H
