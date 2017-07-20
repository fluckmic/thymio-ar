#include "markermodel.h"

// functions for the marker model
// ******************************

// constructor and destructor

MarkerModel::MarkerModel(){

    MarkerModelMonitor *monitor = new MarkerModelMonitor;
    monitor->moveToThread(&monitorThread);

    connect(&monitorThread, &QThread::finished, monitor, &QObject::deleteLater);

    qRegisterMetaType<Timestamp>(); qRegisterMetaType<qPair>(); qRegisterMetaType<std::string>();
    connect(this, &MarkerModel::transformationUpdate, monitor, &MarkerModelMonitor::monitorLinkUpdate);

    // register which link you want to monitor
    monitor->registerLinkUpdateToMonitor(camID, worldID);
    monitor->registerLinkUpdateToMonitor(camID, orangeHouseID);
    monitor->registerLinkUpdateToMonitor(camID, adaHouseID);

    monitorThread.start();
}

// main functions for the qml interface

void MarkerModel::updateLinkNow(const QString &srcFrame, const QString &destFrame, const QMatrix4x4 &trans, float conf){

    // preprocessing which is anyway necessary
    Timestamp tsNow = std::chrono::high_resolution_clock::now();
    qPair qp = matrix2qPair(trans);
    std::string srcF = srcFrame.toStdString(); std::string destF = destFrame.toStdString();

    // signal to monitor
    emit transformationUpdate(srcF, destF, tsNow, qp, conf);

    //                                                ratioLenT   distanceNormalT     distanceNormalPointR    distanceNormalR
    QVector4D thresholdVectorEquality = QVector4D(    0.2,        0.2,                0.2,                    0.3);
    double thresholdConfidence = 0.6;

    // compare transformation update with updated link
    StampedAndRatedTransformation lastUpdate;
    try {
        lastUpdate = getLink(srcF, destF, tsNow);
    } catch(NoSuchLinkFoundException e) { /* no update yet */ }

    QVector4D diff = MarkerModel::compareqPair(qp, qPair{lastUpdate.qRot, lastUpdate.qTra});

    bool equalT = diff.x() < thresholdVectorEquality.x() && diff.y() < thresholdVectorEquality.y() &&
                  diff.z() < thresholdVectorEquality.z() && diff.w() < thresholdVectorEquality.w();
    // if the two transformations are "equal" we update transmem with the average of the two
    if(equalT)
        registerLink(srcF, destF, tsNow, avgQuaternions(lastUpdate.qRot, qp.first), avgQuaternions(lastUpdate.qTra, qp.second));
    // if the two transformations aren't "equal" but the confidence is "high enough" we update transmem with the new transformation
    else if(conf >= thresholdConfidence)
        registerLink(srcF, destF, tsNow, qp.first, qp.second);

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

QVector4D MarkerModel::compareqPair(const qPair &qp1, const qPair &qp2){

    double toRad = M_PI/180;

    /* split each transformation matrix in a translation vector vTra,
       a rotation axis vRotA and an angle fAngl. */

    QVector3D vTra1 = QVector3D(qp1.second.x(), qp1.second.y(), qp1.second.z()),
              vTra2 = QVector3D(qp2.second.x(), qp2.second.y(), qp2.second.z());

    QVector3D vRotA1, vRotA2; float fAngl1, fAngl2;

    qp1.first.getAxisAndAngle(&vRotA1, &fAngl1);
    qp2.first.getAxisAndAngle(&vRotA2, &fAngl2);

    if(vRotA1.z() < 0 ){ vRotA1 *= -1; fAngl1 = 360 - fAngl1; }
    if(vRotA2.z() < 0 ){ vRotA2 *= -1; fAngl2 = 360 - fAngl2; }

    // calculate ratio between the length of the two translation vectors and map them to the interval [0,2]
    double ratioLenT;
    double lenT1 = vTra1.length(), lenT2 = vTra2.length();
    if(lenT1 == 0 && lenT2 == 0)
        ratioLenT = 0;
    else if(lenT1 == 0 || lenT2 == 0)
        ratioLenT = 2;
    else
        ratioLenT = lenT2 > lenT1 ? (1-(lenT1/lenT2))*2. : (1-(lenT2/lenT1))*2.;

    // calculate the distance between the two normalized translation vectors
    vTra1.normalize(); vTra2.normalize();
    double distanceNormalT = fabs((vTra1 - vTra2).length());

    // calculate the distance between a point perpendicular to the rotation axis
    // and this point rotated by the angle difference
    double distanceNormalPointR = sqrt(2*(1-cos((fAngl1-fAngl2)*toRad)));

    // calculate the distance between the two rotation vectors
    vRotA1.normalize(); vRotA2.normalize();
    double distanceNormalR = fabs((vRotA1 - vRotA2).length());

                        //x             //y                 //z                     //w
    return QVector4D(   ratioLenT,     distanceNormalT,    distanceNormalPointR,   distanceNormalR);
}

QQuaternion MarkerModel::avgQuaternions(const QQuaternion &q1, const QQuaternion &q2){
    return QQuaternion( (q1.scalar() + q2.scalar()) / 2.,
                        (q1.x() + q2.x()) / 2.,
                        (q1.y() + q2.y()) / 2.,
                        (q1.z() + q2.z()) / 2.
                       );
}

// functions for the marker model monitor
// **************************************

// functions for monitoring
void MarkerModelMonitor::registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame){

    if(srcFrame == destFrame)
        return;                 // invalid input

    std::string linkID = srcFrame < destFrame ? srcFrame+destFrame : destFrame+srcFrame;

    if(monitoredLinkUpdates.find(linkID) != monitoredLinkUpdates.end())
        return;                 // link is already monitored

    monitoredLinkUpdates.insert({linkID, std::list<LinkUpdate>()});
    monitoredLinkIdentifier.insert({linkID, {srcFrame, destFrame}});

    return;
}

void MarkerModelMonitor::monitorLinkUpdate(const std::string &srcFrame, const std::string &destFrame, const Timestamp &ts,
                                           const qPair &transf, const float &conf) {

   // check if link update is monitored
   if(srcFrame == destFrame)
       return;

  std::string linkID = srcFrame < destFrame ? srcFrame+destFrame : destFrame+srcFrame;

  auto iter2monitoredLinkUpdates = monitoredLinkUpdates.find(linkID);
  if(iter2monitoredLinkUpdates == monitoredLinkUpdates.end())
      return;                 // link is not monitored

  // add entry raw to the corresponding container
    std::list<LinkUpdate> &refToUpdates = ((*iter2monitoredLinkUpdates).second);
  if(refToUpdates.size() < MAX_NUMBER_OF_MONITORED_UPDATES_PER_LINK)
      refToUpdates.push_back(LinkUpdate{ts, transf, conf});
}

void MarkerModelMonitor::registerTransformationToMonitor(const std::string &transID){

    if(monitoredTransformation.find(transID) != monitoredTransformation.end())
        return;                 // transformation already monitored

    monitoredTransformation.insert({transID, std::list<TransformationUpdate>()});
    return;
}

void MarkerModelMonitor::monitorTransformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                                                     const float &avgLinkQuality, const float &avgDistanceToEntry){

    auto iter2monitoredTransformations = monitoredTransformation.find(transID);
    if(iter2monitoredTransformations == monitoredTransformation.end())
        return;             // transformation is not monitored

    // add entry to the corresponding container
    std::list<TransformationUpdate> &refToTransf = ((*iter2monitoredTransformations).second);
    if(refToTransf.size() < MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION)
        refToTransf.push_back(TransformationUpdate{ts, MarkerModel::matrix2qPair(trans), avgLinkQuality, avgDistanceToEntry});
}

// implementation of the different analysis

void LinkUpdateAnalysis::doAnalysis(std::list<LinkUpdate> &input){

    // we need at least two entries to do this analysis
    if(input.size() < 2)
        return;

    // sort list
    auto comperator = ([](const LinkUpdate &lu1, const LinkUpdate &lu2) { return lu1.time > lu2.time;});
    input.sort(comperator);

    Timestamp tZero = input.back().time;

    auto iter = input.begin();
    LinkUpdate preLu = (*iter), curLu;
    iter++;
    while(iter != input.end()){
        curLu = (*iter);
        QVector4D diff = MarkerModel::compareqPair(preLu.transformation, curLu.transformation);
        results.push_front(
              LinkAnalysisSingleResult{
                    (std::chrono::duration_cast<std::chrono::milliseconds>(curLu.time - tZero)).count(),
                    curLu.transformation,
                    curLu.confidence,
                    diff.x(),
                    diff.y(),
                    diff.z(),
                    diff.w()
              }
       );
       preLu = curLu;
       iter++;
    }
}

void LinkUpdateFixAnalysis::doAnalysis(std::list<LinkUpdate> &input){

    // sort list
    auto comperator = ([](const LinkUpdate &lu1, const LinkUpdate &lu2) { return lu1.time > lu2.time;});
    input.sort(comperator);

    Timestamp tZero = input.back().time;

    for(LinkUpdate &l : input){
        QVector4D diff = MarkerModel::compareqPair(l.transformation, fixT);
        results.push_front(
              LinkAnalysisSingleResult{
                    (std::chrono::duration_cast<std::chrono::milliseconds>(l.time - tZero)).count(),
                    l.transformation,
                    l.confidence,
                    diff.x(),
                    diff.y(),
                    diff.z(),
                    diff.w()
              }
       );
    }
}

void TransformationUpdateAnalysis::doAnalysis(std::list<TransformationUpdate> &input) {

    // we need at least two entries to do this analysis
    if(input.size() < 2)
        return;

    // sort list
    auto comperator = ([](const TransformationUpdate &lu1, const TransformationUpdate &lu2) { return lu1.time > lu2.time;});
    input.sort(comperator);

    Timestamp tZero = input.back().time;

    auto iter = input.begin();
    TransformationUpdate preTu = (*iter), curTu;
    iter++;
    while(iter != input.end()){
        preTu = (*iter);
        QVector4D diff = MarkerModel::compareqPair(preTu.transformation, curTu.transformation);
        results.push_front(
              TransformationUpdateAnalysisSingleResult{
                    (std::chrono::duration_cast<std::chrono::milliseconds>(curTu.time - tZero)).count(),
                    curTu.transformation,
                    diff.x(),
                    diff.y(),
                    diff.z(),
                    diff.w(),
                    curTu.avgLinkQuality,
                    curTu.avgDistanceToEntry
              }
       );
       preTu = curTu;
       iter++;
    }

}


// helper functions

void MarkerModelMonitor::writeAnalysisToFile(Analysis &analysis, const QString &path){

    //TODO: rework!

    const QString dSep = ",", newLine = "\n";
    QString suffixFilename = ""; QString prefixData = "";

    QDateTime currentTime = QDateTime::currentDateTime();

    // generate appropriate path and prefix
    if(typeid(analysis) == typeid(LinkUpdateFixAnalysis)){

        LinkUpdateFixAnalysis a = dynamic_cast<LinkUpdateFixAnalysis&>(analysis);

        suffixFilename += "Link_Update_Fix_Analysis_" + a.srcFrame + "_" + a.destFrame + "_";

        prefixData += currentTime.toString("dd,MM,yy,HH,mm,ss") + newLine
                   +  "Link Update Fix Analysis" + dSep + a.srcFrame + dSep + a.destFrame + newLine
                   +  QString::number(a.fixT.first.scalar()) + dSep + QString::number(a.fixT.first.x()) + dSep
                   +  QString::number(a.fixT.first.y()) + dSep + QString::number(a.fixT.first.z()) + dSep
                   +  QString::number(a.fixT.second.scalar()) + dSep + QString::number(a.fixT.second.x()) + dSep
                   +  QString::number(a.fixT.second.y()) + dSep + QString::number(a.fixT.second.z()) + newLine;

    }
    else if(typeid(analysis) == typeid(LinkUpdateAnalysis)){

        LinkUpdateAnalysis a = dynamic_cast<LinkUpdateAnalysis&>(analysis);

        suffixFilename += "Link_Update_Analysis_" + a.srcFrame + "_" + a.destFrame + "_";

        prefixData += currentTime.toString("dd,MM,yy,HH,mm,ss") + newLine
                   +  "Link Update Analysis" + dSep + a.srcFrame + dSep + a.destFrame + newLine;

    }
    else if(typeid(analysis) == typeid(TransformationUpdateAnalysis)){}
    else {
        // unknown analysis
        return;
    }

    QFile file(path + suffixFilename +  currentTime.toString("ddMMyy_HHmmss") + ".m");
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << file.errorString();
        return;
    }

    QTextStream out(&file);
    out << prefixData;

    if(typeid(analysis) == typeid(LinkUpdateAnalysis) || typeid(analysis) == typeid(LinkUpdateFixAnalysis)){
        LinkUpdateAnalysis a = dynamic_cast<LinkUpdateAnalysis&>(analysis);
        for(LinkAnalysisSingleResult &r : a.results)
            out << r.toCSVString() << newLine;
    }
    else if( typeid(analysis) == typeid(TransformationUpdateAnalysis)) {
        TransformationUpdateAnalysis a = dynamic_cast<TransformationUpdateAnalysis&>(analysis);
        for(TransformationUpdateAnalysisSingleResult &r : a.results)
            out << r.toCSVString() << newLine;
    }

    file.close();
    if(file.error()){
        qDebug() << file.errorString();
        return;
    }

}
