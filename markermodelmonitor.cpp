#include "markermodelmonitor.h"

// functions for monitoring

void MarkerModelMonitor::registerLinkUpdateToMonitor(const std::string &srcFrame, const std::string &destFrame){

    if(srcFrame == destFrame)
        return;                 // invalid input

    std::string linkID = srcFrame < destFrame ? srcFrame+destFrame : destFrame+srcFrame;

    if(monitoredLinkUpdates.find(linkID) != monitoredLinkUpdates.end())
        return;                 // link is already monitored

    monitoredLinkUpdates.insert({linkID, std::list<LinkUpdate>()});

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

  /* choose what to analyze:
   *
   *    JITTER between updates -> save raw data, sort afterwards, calculate jitter (can be used to compare against smooth output)
   *    comparison against a fix transformation -> direct calculate difference, save conf and difference, sort and output afterwards (better use of conf)
   */

}

// implementation of the different analysis

// Link Update Relative Analysis
void LinkUpdateRelativeAnalysis::doAnalysis(const std::list<LinkUpdate> &input){}

std::string LinkUpdateRelativeAnalysis::singleResult2String(const LinkAnalysisSingleResult &re) { return "dummy"; }

std::string LinkUpdateRelativeAnalysis::getAnalysisType(){
    return "Link Update Relative Analysis";
}

// Link Update Fix Analysis

void LinkUpdateFixAnalysis::doAnalysis(const std::list<LinkUpdate> &input){}

std::string LinkUpdateFixAnalysis::singleResult2String(const LinkAnalysisSingleResult &re) { return "dummy"; }

std::string LinkUpdateFixAnalysis::getAnalysisType(){
    return "Link Update Fix Analysis";
}

// helper functions

void MarkerModelMonitor::writeAnalysisToFile(const Analysis &analysis, const QString &path){}
