#include "markermodel.h"

// functions for the marker model
// ******************************

// constructor and destructor

MarkerModel::MarkerModel(){

    // necessary to allow connections to marker model monitor
    qRegisterMetaType<Timestamp>(); qRegisterMetaType<qPair>(); qRegisterMetaType<std::string>();

    // create a new monitor object
    MarkerModelMonitor* monitor = new MarkerModelMonitor;
    monitor->moveToThread(&monitorThread);

    //connect(&monitorThread, &QThread::finished, monitor, &QObject::deleteLater);

    connect(this, &MarkerModel::linkUpdate, monitor, &MarkerModelMonitor::monitorLinkUpdate);
    connect(this, &MarkerModel::transformationUpdate, monitor, &MarkerModelMonitor::monitorTransformationUpdate);

    connect(this, &MarkerModel::startMonitor, monitor, &MarkerModelMonitor::startMonitoring);
    connect(this, &MarkerModel::stopMonitor, monitor, &MarkerModelMonitor::stopMonitoring);

    connect(this, &MarkerModel::registerLinkUpdateToMonitor, monitor, &MarkerModelMonitor::registerLinkUpdateToMonitor);
    connect(this, &MarkerModel::registerTransformationToMonitor, monitor, &MarkerModelMonitor::registerTransformationToMonitor);

    monitorThread.start();

}

// main functions for the qml interface

void MarkerModel::updateLinkNow(const QString &srcFrame, const QString &destFrame, const QMatrix4x4 &trans, float conf){

    // preprocessing which is anyway necessary
    Timestamp tsNow = std::chrono::high_resolution_clock::now();
    qPair qp = matrix2qPair(trans);
    std::string srcF = srcFrame.toStdString(); std::string destF = destFrame.toStdString();

    // signal to monitor
    emit linkUpdate(srcF, destF, tsNow, qp, conf);

    // compare transformation update with updated link
    StampedAndRatedTransformation lastUpdate;
    try {
        lastUpdate = getLink(srcF, destF, tsNow);
    } catch(NoSuchLinkFoundException e) { /* no update yet */ }

    if(conf >= thConfidenceMarkerUpdate)
        registerLink(srcF, destF, tsNow, qp.first, qp.second, conf);
    else
        try {
        updateLinkQuality(srcF, destF, conf);
    } catch(NoSuchLinkFoundException e) { /* no update yet */ }

}

void MarkerModel::updateModel(){

    // preprocessing which is anyway necessary
    Timestamp tsNow = std::chrono::high_resolution_clock::now();

    // helper lambda
    // TODO: maybe not as lambda
    /* SART == StampedAndRatedTransformation */
    auto SARTMultiplier = [](const StampedAndRatedTransformation &lhs, const StampedAndRatedTransformation &rhs){
        StampedAndRatedTransformation ret;

        ret.time = lhs.time;

        ret.qRot = lhs.qRot * rhs.qRot;
        ret.qTra = lhs.qRot * rhs.qTra * lhs.qRot.conjugated();
        ret.qTra = ret.qTra + lhs.qTra;

        ret.avgLinkQuality = lhs.avgLinkQuality > rhs.avgLinkQuality ? lhs.avgLinkQuality : rhs.avgLinkQuality;
        ret.maxDistanceToEntry = lhs.maxDistanceToEntry > rhs.maxDistanceToEntry ? lhs.maxDistanceToEntry : rhs.maxDistanceToEntry;

        return ret;
    };

    auto SARTAveragerIfEqual = [this](const StampedAndRatedTransformation &lhs, const StampedAndRatedTransformation &rhs){

        bool encodeEqualTransformation = equalTransformation(qPair{lhs.qRot, lhs.qTra}, qPair{rhs.qRot, rhs.qTra});

        if(!encodeEqualTransformation)
            return lhs;


        StampedAndRatedTransformation ret;

        NumbAverages++;
        qDebug() << "averaging.. " << NumbAverages;

        ret.qRot = avgAndNormalizeQuaternions(lhs.qRot, rhs.qRot);
        ret.qTra = avgQuaternions(lhs.qTra, rhs.qTra);
        ret.avgLinkQuality = (lhs.avgLinkQuality + rhs.avgLinkQuality ) / 2;
        ret.maxDistanceToEntry =    (lhs.maxDistanceToEntry + rhs.maxDistanceToEntry) / 2.;
        ret.time = lhs.time;

        return ret;
    };

    auto SARTInverter = [](StampedAndRatedTransformation &lhs){

        lhs.qRot = lhs.qRot.inverted();
        lhs.qTra = -(lhs.qRot*lhs.qTra*lhs.qRot.conjugated());

        return lhs;
    };

    // **************************************************************************/
    // MODEL 1
    // **************************************************************************/

    // fetch all transformations needed
    StampedAndRatedTransformation world2camNowWCM, orangeHouse2camNow, adaHouse2camNow,
                                  world2orangeHouseFix, world2adaHouseFix, world2adaHouseNow,
                                  world2orangeHouseNow;

    try{ world2camNowWCM = getLink(worldID, camID, tsNow); }
    catch(NoSuchLinkFoundException){
        // we require the world center to be present, otherwise no object
        // is displayed
        return;
    }

    try{ orangeHouse2camNow = getLink(orangeHouseID, camID, tsNow); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    try{ adaHouse2camNow = getLink(adaHouseID, camID, tsNow); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    try{ world2orangeHouseFix = getBestLink(worldID, orangeHouseID); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    try{ world2adaHouseFix = getBestLink(worldID, adaHouseID); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    try{ world2orangeHouseNow = getLink(worldID, orangeHouseID, tsNow); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    try{ world2adaHouseNow = getLink(worldID, adaHouseID, tsNow); }
    catch(NoSuchLinkFoundException){ /* no link registered yet */ }

    StampedAndRatedTransformation world2camNowOHM = SARTMultiplier(orangeHouse2camNow, world2orangeHouseFix),
                                  world2camNowAHM = SARTMultiplier(adaHouse2camNow, world2adaHouseFix);

    StampedAndRatedTransformation world2orangeHouseBest = world2orangeHouseNow;
    StampedAndRatedTransformation world2adaHouseBest = world2adaHouseNow;

    // if the confidence for a link is to low, so we set the marker to inactive
    world2camActiveP = !(world2camNowWCM.avgLinkQuality < thConfidenceMarkerActive);
    world2orangeHouseActiveP = !(orangeHouse2camNow.avgLinkQuality < thConfidenceMarkerActive);
    world2adaHouseActiveP = !(adaHouse2camNow.avgLinkQuality < thConfidenceMarkerActive);

    // we sort the transformation from world to camera base depending on maxDistanceToEntry
    // for a fix link, maxDistanceToEntry is the distance to the transformation entry on that link, where
    // the entry is farthermost.
    std::list<StampedAndRatedTransformation> trnsWorld2camNow = {world2camNowWCM, world2camNowOHM, world2camNowAHM};
    auto cmp = [](const StampedAndRatedTransformation &lhs, const StampedAndRatedTransformation &rhs){ return lhs.maxDistanceToEntry < rhs.maxDistanceToEntry; };
    trnsWorld2camNow.sort(cmp);

    StampedAndRatedTransformation world2camBest = trnsWorld2camNow.front(); trnsWorld2camNow.pop_front();
    StampedAndRatedTransformation world2cam2Best = trnsWorld2camNow.front(); trnsWorld2camNow.pop_front();
    StampedAndRatedTransformation world2cam3Best = trnsWorld2camNow.front();

    // if no transformation from world to camera base has a value maxDistanceToEntry which is smaller
    // than the threshold, we cannot calculate a reasonable good transformation for world2cam, so even
    // if the world center marker is partialy visible, we set it to inactive
    if(world2camBest.maxDistanceToEntry > thDistanceToLastUpdate){
        world2camActiveP = false;
    }
    // at least one transformation should be reasonable good, we check if there is another
    // one which is also good enough, if yes we compare the transformation matrices and average
    // them if they encode a similar transformation
    else if(world2cam2Best.maxDistanceToEntry < thDistanceToLastUpdate &&
            world2cam3Best.maxDistanceToEntry > thDistanceToLastUpdate){
        world2camBest = SARTAveragerIfEqual(world2camBest, world2cam2Best);
    }
    // if there is even a third good transformation we also use this one
    else if(world2cam3Best.maxDistanceToEntry < thDistanceToLastUpdate){
        world2camBest = SARTAveragerIfEqual(world2camBest, world2cam2Best);
        world2camBest = SARTAveragerIfEqual(world2camBest, world2cam3Best);
    }

    world2camP = qPair2Matrix(qPair{world2camBest.qRot, world2camBest.qTra});

    world2orangeHouseP = qPair2Matrix(qPair{world2orangeHouseNow.qRot, world2orangeHouseNow.qTra});
    world2adaHouseP = qPair2Matrix(qPair{world2adaHouseNow.qRot, world2adaHouseNow.qTra});

    qDebug() << world2orangeHouseActiveP;

    // **************************************************************************/

    // tell analyzer about new results
      emit transformationUpdate(world2camID, tsNow, world2camP, world2camBest.avgLinkQuality, world2camBest.maxDistanceToEntry);
      emit transformationUpdate(world2orangeHouseID, tsNow, world2orangeHouseP, world2orangeHouseBest.avgLinkQuality, world2orangeHouseBest.maxDistanceToEntry);
      emit transformationUpdate(world2adaHouseID, tsNow, world2adaHouseP, world2adaHouseBest.avgLinkQuality, world2adaHouseBest.maxDistanceToEntry);

    // tell qml that new transformations are available
    emit transformationsUpdated();

}

// monitoring functions for the qml interface

void MarkerModel::startMonitoring(){

    // can add these also to the gui..
    emit registerLinkUpdateToMonitor(camID, worldID);
    emit registerLinkUpdateToMonitor(camID, orangeHouseID);
    emit registerLinkUpdateToMonitor(camID, adaHouseID);

    emit registerTransformationToMonitor(world2camID);
    emit registerTransformationToMonitor(world2orangeHouseID);
    emit registerTransformationToMonitor(world2adaHouseID);

    emit startMonitor();
}

void MarkerModel::stopMonitoring(){
    emit stopMonitor();
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

bool MarkerModel::world2camActive(){
    return world2camActiveP;
}

bool MarkerModel::world2orangeHouseActive(){
    return world2orangeHouseActiveP;
}

bool MarkerModel::world2adaHouseActive(){
    return world2adaHouseActiveP;
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

QQuaternion MarkerModel::avgAndNormalizeQuaternions(const QQuaternion &q1, const QQuaternion &q2){
    QQuaternion ret = QQuaternion( (q1.scalar() + q2.scalar()) / 2.,
                                   (q1.x() + q2.x()) / 2.,
                                   (q1.y() + q2.y()) / 2.,
                                   (q1.z() + q2.z()) / 2.
                                 );

    return ret.normalized();
}

QQuaternion MarkerModel::avgQuaternions(const QQuaternion &q1, const QQuaternion &q2){
    QQuaternion ret = QQuaternion( (q1.scalar() + q2.scalar()) / 2.,
                                   (q1.x() + q2.x()) / 2.,
                                   (q1.y() + q2.y()) / 2.,
                                   (q1.z() + q2.z()) / 2.
                                 );
    return ret;
}

bool MarkerModel::equalTransformation(const qPair &qp1, const qPair &qp2){

    //                                                       ratioLenT   distanceNormalT     distanceNormalPointR    distanceNormalR
    const QVector4D thTransformationEquality = QVector4D(    0.002,      0.002,              0.002,                  0.002);

    QVector4D diff = compareqPair(qp1, qp2);

    return diff.x() < thTransformationEquality.x() && diff.y() < thTransformationEquality.y() &&
           diff.z() < thTransformationEquality.z() && diff.w() < thTransformationEquality.w();

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

   if(!currentlyMonitoring)
       return;

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

void MarkerModelMonitor::startMonitoring() {

        if(currentlyMonitoring)
        return;         // we already monitor

    monitoringStartedAt = std::chrono::high_resolution_clock::now();

    currentlyMonitoring = true;
}

void MarkerModelMonitor::stopMonitoring() {

    if(!currentlyMonitoring)
        return;         // we aren't monitoring right now

    QDateTime currentTime = QDateTime::currentDateTime();
    QString folderPath = PATH + "Analysis_" + currentTime.toString("ddMMyy_HHmmss") + "/";

    // create folder for analyis results
    QDir dir;
    int dirExists = dir.exists(folderPath);
    // if not, create it
    if( !dirExists )
        dir.mkdir(folderPath);

    // do a link update analysis for all monitored link updates
    auto iter = monitoredLinkUpdates.begin();
    while(iter != monitoredLinkUpdates.end()){
        std::string linkID = (*iter).first;
        std::string srcFrame = (monitoredLinkIdentifier.at(linkID)).first;
        std::string destFrame = (monitoredLinkIdentifier.at(linkID)).second;

        LinkUpdateAnalysis lua = LinkUpdateAnalysis(monitoringStartedAt, QString::fromStdString(srcFrame), QString::fromStdString(destFrame));
        lua.doAnalysis(monitoredLinkUpdates.at(linkID));
        writeSingleAnalysisToFile(lua, folderPath);
        iter++;
    }

    // do a transformation update analysis for all monitored transformations
    auto iter2 = monitoredTransformation.begin();
    while(iter2 != monitoredTransformation.end()){
        std::string transID = (*iter2).first;

        TransformationUpdateAnalysis tua = TransformationUpdateAnalysis(monitoringStartedAt, QString::fromStdString(transID));
        tua.doAnalysis((*iter2).second);
        writeSingleAnalysisToFile(tua, folderPath);
        iter2++;
    }

    // clean up all container for the next monitoring session
    monitoredLinkIdentifier.clear();
    monitoredLinkUpdates.clear();
    monitoredTransformation.clear();

    currentlyMonitoring = false;
}

void MarkerModelMonitor::monitorTransformationUpdate(const std::string &transID, const Timestamp &ts, const QMatrix4x4 &trans,
                                                     const float &avgLinkQuality, const float &maxDistanceToEntry){

    if(!currentlyMonitoring)
        return;

    auto iter2monitoredTransformations = monitoredTransformation.find(transID);
    if(iter2monitoredTransformations == monitoredTransformation.end())
        return;             // transformation is not monitored

    // add entry to the corresponding container
    std::list<TransformationUpdate> &refToTransf = ((*iter2monitoredTransformations).second);
    if(refToTransf.size() < MAX_NUMBER_OF_MONITORED_UPDATES_PER_TRANFORMATION)
        refToTransf.push_back(TransformationUpdate{ts, MarkerModel::matrix2qPair(trans), avgLinkQuality, maxDistanceToEntry});
}

// implementation of the different analysis

void LinkUpdateAnalysis::doAnalysis(std::list<LinkUpdate> &input){

    // we need at least two entries to do this analysis
    if(input.size() < 2)
        return;

    // sort list
    auto comperator = ([](const LinkUpdate &lu1, const LinkUpdate &lu2) { return lu1.time > lu2.time;});
    input.sort(comperator);

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

    auto iter = input.begin();
    TransformationUpdate preTu = (*iter), curTu;
    iter++;
    while(iter != input.end()){
        curTu = (*iter);
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
                    curTu.maxDistanceToEntry
              }
       );
       preTu = curTu;
       iter++;
    }

}

// helper functions

void MarkerModelMonitor::writeSingleAnalysisToFile(Analysis &analysis, const QString &path){

    const QString dSep = ",", newLine = "\n";
    QString suffixFilename = ""; QString prefixData = "";

    // generate appropriate path and prefix
    if(typeid(analysis) == typeid(LinkUpdateFixAnalysis)){

        LinkUpdateFixAnalysis a = dynamic_cast<LinkUpdateFixAnalysis&>(analysis);

        suffixFilename += "Link_Update_Fix_Analysis_" + a.srcFrame + "_" + a.destFrame;

        prefixData += "Link Update Fix Analysis" + dSep + a.srcFrame + dSep + a.destFrame + newLine
                   +  QString::number(a.fixT.first.scalar()) + dSep + QString::number(a.fixT.first.x()) + dSep
                   +  QString::number(a.fixT.first.y()) + dSep + QString::number(a.fixT.first.z()) + dSep
                   +  QString::number(a.fixT.second.scalar()) + dSep + QString::number(a.fixT.second.x()) + dSep
                   +  QString::number(a.fixT.second.y()) + dSep + QString::number(a.fixT.second.z()) + newLine;

    }
    else if(typeid(analysis) == typeid(LinkUpdateAnalysis)){

        LinkUpdateAnalysis a = dynamic_cast<LinkUpdateAnalysis&>(analysis);

        suffixFilename += "Link_Update_Analysis_" + a.srcFrame + "_" + a.destFrame;

        prefixData += "Link Update Analysis" + dSep + a.srcFrame + dSep + a.destFrame + newLine;

    }
    else if(typeid(analysis) == typeid(TransformationUpdateAnalysis)){

        TransformationUpdateAnalysis a = dynamic_cast<TransformationUpdateAnalysis&>(analysis);

        suffixFilename += "Transformation_Update_Analysis_" + a.transID;

        prefixData += "Transformation Update Analysis" + dSep + a.transID + newLine;
    }
    else {
        // unknown analysis
        return;
    }

    QFile file(path + suffixFilename + ".m");
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
