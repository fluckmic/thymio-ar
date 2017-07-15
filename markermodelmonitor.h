#ifndef MARKERMODELMONITOR_H
#define MARKERMODELMONITOR_H

#include <QObject>
#include <QDebug>
#include <QQuaternion>
#include <QVector4D>

#include <unordered_map>
#include <list>

typedef std::chrono::time_point<std::chrono::high_resolution_clock> Timestamp;
typedef std::pair<QQuaternion, QQuaternion> qPair;

struct LinkUpdate {
    Timestamp time;
    qPair transformation;
    float confidence;
};

struct TransformationUpdate : public LinkUpdate {
    float avgTimeDiff;
    // fields for time, transformation and avgLinkQuality are inherited
};

struct LinkAnalysisSingleResult {};

struct Analysis{
    virtual std::string getAnalysisType() = 0;
    Timestamp analysedAt;
};

struct LinkUpdateAnalysis : Analysis {
    virtual void doAnalysis(const std::list<LinkUpdate> &input) = 0;
    virtual std::string singleResult2String(const LinkAnalysisSingleResult &re) = 0;

    const std::string srcFrame;
    const std::string destFrame;
    std::list<LinkAnalysisSingleResult> results;
};

struct LinkUpdateRelativeAnalysis : LinkUpdateAnalysis {
    virtual void doAnalysis(const std::list<LinkUpdate> &input) override;
    virtual std::string singleResult2String(const LinkAnalysisSingleResult &re) override;
    virtual std::string getAnalysisType() override;
};

struct LinkUpdateFixAnalysis: LinkUpdateAnalysis {
    virtual void doAnalysis(const std::list<LinkUpdate> &input) override;
    virtual std::string singleResult2String(const LinkAnalysisSingleResult &re) override;
    virtual std::string getAnalysisType() override;
    qPair fixT;
};

class MarkerModelMonitor : public QObject {

    Q_OBJECT

public:

    ~MarkerModelMonitor(){}

    void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame);
    void registerTransformationToMonitor(const QString &transID);

    void writeAnalysisToFile(const Analysis &analysis, const QString &path);

public slots:
    void monitorLinkUpdate(const std::string &srcFrame, const std::string &destFrame, const Timestamp &ts,
                           const qPair &transf, const float &conf);

protected:
    std::unordered_map<std::string, std::list<LinkUpdate> > monitoredLinkUpdates;

    const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_LINK = 1000000;
};

#endif // MARKERMODELMONITOR_H
