#include "markermodel.h"

// main functions for the qml interface

void MarkerModel::updateLinkNow(const QString &srcFrame, const QString &destFrame, const QMatrix4x4 &trans, float conf){

    // preprocessing which is anyway necessary
    Timestamp tsNow = std::chrono::high_resolution_clock::now();
    qPair qp = matrix2qPair(trans);
    std::string srcF = srcFrame.toStdString(); std::string destF = destFrame.toStdString();

    // signal to analyzer
    emit transformationUpdate(srcF, destF, tsNow, qp, conf);

    // STAGE 0

    registerLink(srcF, destF, tsNow, qp.first, qp.second);

    // compare last saved transformation matrix and the current

        // save average in transmem if "equal"

    // check if confidence "high enough"

        // save entry in transmem if it is

    // update average confidence

}

void MarkerModel::updateModel(){

    // preprocessing which is anyway necessary
    Timestamp tsNow = std::chrono::high_resolution_clock::now();

    // STAGE 0
    /* SART == StampedAndRatedTransformation */
    StampedAndRatedTransformation world2camSART = getLink(worldID, camID, tsNow),
                                  world2orangeHouseSART = getLink(worldID, orangeHouseID, tsNow),
                                  world2adaHouseSART = getLink(worldID, adaHouseID, tsNow);

    world2camP = qPair2Matrix(qPair{world2camSART.qRot, world2camSART.qTra});
    world2orangeHouseP = qPair2Matrix(qPair{world2orangeHouseSART.qRot, world2orangeHouseSART.qTra});
    world2adaHouseP = qPair2Matrix(qPair{world2adaHouseSART.qRot, world2adaHouseSART.qTra});

    // tell analyzer about new results
    // fire signal to analyzer that transformations are updated

    // tell qml that new transformations are available
    emit transformationsUpdated();
}


// helper functions for the qml interface

QMatrix4x4 MarkerModel::world2cam(){
    return world2camP;
}

QMatrix4x4 MarkerModel::world2orangeHouse(){
    return world2orangeHouseP;
}

QMatrix4x4 MarkerModel::world2adaHouse(){
    return world2adaHouseP;
}

// helper functions

QMatrix4x4 MarkerModel::qPair2Matrix(const qPair &qp){

    QMatrix4x4 ret(qp.first.toRotationMatrix());
    ret(0,3) = qp.second.x();   ret(1,3) = qp.second.y(); ret(2,3) = qp.second.z();
    return ret;

}

qPair MarkerModel::matrix2qPair(const QMatrix4x4 &m){

    float data[]{ m(0,0),m(0,1),m(0,2),
                  m(1,0),m(1,1),m(1,2),
                  m(2,0),m(2,1),m(2,2)
                };

    QMatrix3x3 rM(data);

    return qPair{QQuaternion::fromRotationMatrix(rM), QQuaternion(0, m(0,3), m(1,3), m(2,3))};
}

