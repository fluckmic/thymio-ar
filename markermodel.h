#ifndef MARKERMODEL_H
#define MARKERMODEL_H

#include <QObject>
#include <QQmlListProperty>
#include <QThread>

#include "transmem/transmem.h"
#include "markermodelmonitor.h"

typedef std::pair<QQuaternion, QQuaternion> qPair;

Q_DECLARE_METATYPE(Timestamp)
Q_DECLARE_METATYPE(qPair)
Q_DECLARE_METATYPE(std::string)

class MarkerModel : public QObject, public TransMem {

    Q_OBJECT
    QThread monitorThread;

public:

    MarkerModel(){
        MarkerModelMonitor *monitor = new MarkerModelMonitor;
        monitor->moveToThread(&monitorThread);

        connect(&monitorThread, &QThread::finished, monitor, &QObject::deleteLater);

        qRegisterMetaType<Timestamp>(); qRegisterMetaType<qPair>(); qRegisterMetaType<std::string>();
        connect(this, &MarkerModel::transformationUpdate, monitor, &MarkerModelMonitor::monitorLinkUpdate);

        // register which link you want to monitor
        monitor->registerLinkUpdateToMonitor(camID, worldID);

        monitorThread.start();
    }

    ~MarkerModel(){
        monitorThread.quit();
        monitorThread.wait();
    }

    // calculated transformations
    Q_PROPERTY(QMatrix4x4 world2cam READ world2cam NOTIFY transformationsUpdated)
    Q_PROPERTY(QMatrix4x4 world2orangeHouse READ world2orangeHouse NOTIFY transformationsUpdated)
    Q_PROPERTY(QMatrix4x4 world2adaHouse READ world2adaHouse NOTIFY transformationsUpdated)

    // function to update transmem
    Q_INVOKABLE void updateLinkNow(const QString& srcFrame, const QString& destFrame, const QMatrix4x4& trans, float conf);
    Q_INVOKABLE void updateModel();

private:
    // getter for the qml attributes
    QMatrix4x4 world2cam();
    QMatrix4x4 world2orangeHouse();
    QMatrix4x4 world2adaHouse();

    // helper functions
    qPair matrix2qPair(const QMatrix4x4& m);
    QMatrix4x4 qPair2Matrix(const qPair& qp);

    double compareqPair(const qPair& qp1, const qPair& qp2);
    qPair avgqPair(const qPair& qp1, const qPair& qp2);

    // confidence mapping
    float f(const float& conf){ return conf > 1 ? 1 : conf < 0 ? 0 : conf; }

    // private members
    QMatrix4x4 world2camP;
    QMatrix4x4 world2orangeHouseP;
    QMatrix4x4 world2adaHouseP;

    // private identifier
    // -> have to match identifier in qml!
    const std::string camID = "cam";
    const std::string worldID = "world";
    const std::string orangeHouseID = "orangeHouse";
    const std::string adaHouseID = "adaHouse";

signals:
    void transformationUpdate(const std::string &srcFrame, const std::string &dstFrame, const Timestamp &ts,
                              const qPair &transf, const float &conf);
    void transformationsUpdated();

};

#endif // MARKERMODEL_H
