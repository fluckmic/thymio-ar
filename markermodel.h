#ifndef MARKERMODEL_H
#define MARKERMODEL_H

#include <QObject>
#include <QQmlListProperty>
#include <QThread>
#include <QDebug>
#include <QQuaternion>
#include <QVector4D>
#include <QDateTime>
#include <QFile>
#include <QDir>
#include <chrono>
#include <unordered_map>
#include <list>

#include "transmem/transmem.h"

typedef std::pair<QQuaternion, QQuaternion> qPair;

Q_DECLARE_METATYPE(Timestamp)
Q_DECLARE_METATYPE(qPair)
Q_DECLARE_METATYPE(std::string)

class MarkerModelMonitor;

// Marker model //
class MarkerModel : public QObject, public TransMem {

    Q_OBJECT

public:

    MarkerModel();

    ~MarkerModel(){

        stopMonitoring();

        monitorThread.quit();
        monitorThread.wait();
    }

    // calculated transformations
    Q_PROPERTY(QMatrix4x4 world2cam READ world2cam NOTIFY transformationsUpdated)
    Q_PROPERTY(QMatrix4x4 world2orangeHouse READ world2orangeHouse NOTIFY transformationsUpdated)
    Q_PROPERTY(QMatrix4x4 world2adaHouse READ world2adaHouse NOTIFY transformationsUpdated)

    Q_PROPERTY(bool world2camActive READ world2camActive NOTIFY transformationsUpdated)
    Q_PROPERTY(bool world2adaHouseActive READ world2adaHouseActive NOTIFY transformationsUpdated)
    Q_PROPERTY(bool world2orangeHouseActive READ world2orangeHouseActive NOTIFY transformationsUpdated)


    // function to update transmem
    Q_INVOKABLE void updateLinkNow(const QString& srcFrame, const QString& destFrame, const QMatrix4x4& trans, float conf);
    Q_INVOKABLE void updateModel();

    // functions to start and stop monitoring
    Q_INVOKABLE void startMonitoring();
    Q_INVOKABLE void stopMonitoring();

    // helper functions
    static qPair matrix2qPair(const QMatrix4x4& m);
    static QMatrix4x4 qPair2Matrix(const qPair& qp);
    static QVector4D compareqPair(const qPair& qp1, const qPair& qp2);
    static bool equalTransformation(const qPair &qp1, const qPair &qp2);

private:

    int NumbAverages = 0;

    // getter for the qml attributes
    QMatrix4x4 world2cam();
    QMatrix4x4 world2orangeHouse();
    QMatrix4x4 world2adaHouse();

    bool world2camActive();
    bool world2orangeHouseActive();
    bool world2adaHouseActive();

    // confidence mapping
    float f(const float& conf){ return conf > 1 ? 1 : conf < 0 ? 0 : conf; }

    // empiric values of the model
    const double thConfidenceMarkerActive = 0.4;
    const double thConfidenceMarkerUpdate = 0.6;
    const double thDistanceToLastUpdate = 70;       // in ms

    // private members
    QMatrix4x4 world2camP;
    QMatrix4x4 world2orangeHouseP;
    QMatrix4x4 world2adaHouseP;

    bool world2camActiveP;
    bool world2orangeHouseActiveP;
    bool world2adaHouseActiveP;

    // private identifier
    // -> have to match identifier in qml!
    const std::string camID = "cam";
    const std::string worldID = "world";
    const std::string orangeHouseID = "orangeHouse";
    const std::string adaHouseID = "adaHouse";

    const std::string world2camID = "world2cam";
    const std::string world2orangeHouseID = "world2orangeHouse";
    const std::string world2adaHouseID = "world2adaHouse";

    QThread monitorThread;

signals:
    // signals for the monitor
    void linkUpdate(const std::string &srcFrame, const std::string &dstFrame, const Timestamp &ts,
                    const qPair &transf, const float &conf);

    void transformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                              const float &avgLinkQuality, const float &avgDistanceToEntry);

    void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame);
    void registerTransformationToMonitor(const std::string &transID);

    void startMonitor();
    void stopMonitor();

    // signals for qml model
    void transformationsUpdated();

protected:
    QQuaternion avgAndNormalizeQuaternions(const QQuaternion &q1, const QQuaternion &q2);
    QQuaternion avgQuaternions(const QQuaternion &q1, const QQuaternion &q2);
};

// Container needed in marker model monitor to store monitored data
// and calculated results of monitored data

// stores a single update of a link direct from the tracker
struct LinkUpdate {
    Timestamp time;
    qPair transformation;
    float confidence;
};

// stores a single update of a transformation used to draw an object
struct TransformationUpdate  {
    Timestamp time;
    qPair transformation;
    float avgLinkQuality;
    float maxDistanceToEntry;
};

// basic type of an analysis
struct Analysis{
    virtual ~Analysis(){}

    Analysis(const Timestamp &tZero)
    : tZero(tZero)
    {}

    // allows to compare different analysis
    Timestamp tZero;
};

// basic type of an analysis result
struct SingleAnalysisResult{ virtual QString toCSVString() = 0; };

// stores a single result of a link update analysis
struct LinkAnalysisSingleResult : SingleAnalysisResult {

    LinkAnalysisSingleResult(   unsigned int tms, qPair tf, double conf,
                                double ratioLenT, double distanceNormalT,
                                double distanceNormalPointR, double distanceNormalR)
    : tms(tms)
    , tf(tf)
    , conf(conf)
    , ratioLenT(ratioLenT)
    , distanceNormalT(distanceNormalT)
    , distanceNormalPointR(distanceNormalPointR)
    , distanceNormalR(distanceNormalR)
    {}

    unsigned int tms;
    qPair tf;
    double conf;
    double ratioLenT;
    double distanceNormalT;
    double distanceNormalPointR;
    double distanceNormalR;

    virtual QString toCSVString() override {
        const QString c = ",";
        return QString::number(tms)+c+QString::number(tf.first.scalar())+c+
               QString::number(tf.first.x())+c+QString::number(tf.first.y())+c+
               QString::number(tf.first.z())+c+QString::number(tf.second.scalar())+c+
               QString::number(tf.second.x())+c+QString::number(tf.second.y())+c+
               QString::number(tf.second.z())+c+QString::number(ratioLenT)+c+
               QString::number(distanceNormalT)+c+QString::number(distanceNormalPointR)+c+
               QString::number(distanceNormalR)+c+QString::number(conf);
    }
};

// stores a single result of a transformation update analysis
struct TransformationUpdateAnalysisSingleResult : SingleAnalysisResult {

    TransformationUpdateAnalysisSingleResult( unsigned int tms, qPair tf, double ratioLenT,
                                              double distanceNormalT, double distanceNormalPointR,
                                              double distanceNormalR,
                                              double avgLinkQuality, double avgDistanceToEntry)
    : tms(tms)
    , tf(tf)
    , ratioLenT(ratioLenT)
    , distanceNormalT(distanceNormalT)
    , distanceNormalPointR(distanceNormalPointR)
    , distanceNormalR(distanceNormalR)
    , avgLinkQuality(avgLinkQuality)
    , avgDistanceToEntry(avgDistanceToEntry)
    {}

    unsigned int tms;
    qPair tf;
    double ratioLenT;
    double distanceNormalT;
    double distanceNormalPointR;
    double distanceNormalR;
    double avgLinkQuality;
    double avgDistanceToEntry;

    virtual QString toCSVString() override {
        const QString c = ",";
        return QString::number(tms)+c+QString::number(tf.first.scalar())+c+
               QString::number(tf.first.x())+c+QString::number(tf.first.y())+c+
               QString::number(tf.first.z())+c+QString::number(tf.second.scalar())+c+
               QString::number(tf.second.x())+c+QString::number(tf.second.y())+c+
               QString::number(tf.second.z())+c+QString::number(ratioLenT)+c+
               QString::number(distanceNormalT)+c+QString::number(distanceNormalPointR)+c+
               QString::number(distanceNormalR)+c+QString::number(avgLinkQuality)+c+
               QString::number(avgDistanceToEntry);
    }
};

struct LinkUpdateAnalysis : Analysis {

    LinkUpdateAnalysis(const Timestamp &tsZero, const QString &srcFrame, const QString &destFrame)
    : Analysis(tsZero)
    , srcFrame(srcFrame)
    , destFrame(destFrame)
    {}

    void doAnalysis(std::list<LinkUpdate> &input);

    const QString srcFrame;
    const QString destFrame;

    std::list<LinkAnalysisSingleResult> results;
};

struct LinkUpdateFixAnalysis : LinkUpdateAnalysis {

    LinkUpdateFixAnalysis(const Timestamp &tsZero, const QString &srcFrame, const QString &destFrame, const qPair &fixT)
    : LinkUpdateAnalysis(tsZero, srcFrame, destFrame)
    , fixT(fixT)
    {}

    //override
    void doAnalysis(std::list<LinkUpdate> &input);

    qPair fixT;
};

struct TransformationUpdateAnalysis : Analysis {

    TransformationUpdateAnalysis(const Timestamp &tsZero, const QString &transID)
    : Analysis(tsZero)
    , transID(transID)
    {}

    void doAnalysis(std::list<TransformationUpdate> &input);

    const QString transID;

    std::list<TransformationUpdateAnalysisSingleResult> results;
};

// Marker model monitor //

class MarkerModelMonitor : public QObject {

    Q_OBJECT

public:

    void writeSingleAnalysisToFile(Analysis &analysis, const QString &path);



public slots:
    void monitorLinkUpdate(const std::string &srcFrame, const std::string &destFrame, const Timestamp &ts,
                           const qPair &transf, const float &conf);

    void monitorTransformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                                     const float &avgLinkQuality, const float &maxDistanceToEntry);

    void registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame);
    void registerTransformationToMonitor(const std::string &transID);

    void startMonitoring();
    void stopMonitoring();

protected:
    // container for the monitored link updates
    std::unordered_map<std::string, std::list<LinkUpdate> > monitoredLinkUpdates;
    std::unordered_map<std::string, std::pair<std::string, std::string> > monitoredLinkIdentifier;

    // container for the monitored transformation
    std::unordered_map<std::string, std::list<TransformationUpdate> > monitoredTransformation;

    const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_LINK = 1000000;
    const unsigned int MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION = 1000000;

    const QString PATH = "../thymio-ar-demo/analysis/";

    Timestamp monitoringStartedAt;

    bool currentlyMonitoring = false;

};

#endif // MARKERMODEL_H
