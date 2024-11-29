/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      interface_lane.hpp
 * @brief     Lane structure, import and export
 * 
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : add interface structs
 */

#pragma once

#include <string> // std::string
#include <vector> // std::vector
#include <boost/filesystem.hpp> // boost::filesystem
#include <ctime>
#include <fstream>  // std::ifstream
#include <iomanip>  // std::setw
#include <iostream> // std::cout
#include <sstream>
#include <stdio.h>  // remove

struct SLanePoint {
    double m_dPtX_m;
    double m_dPtY_m;

public:
    SLanePoint() 
        : m_dPtX_m(0.0), m_dPtY_m(0.0) {}
    SLanePoint(double dPtX_m, double dPtY_m)
        : m_dPtX_m(dPtX_m), m_dPtY_m(dPtY_m) {}
    SLanePoint(const SLanePoint &other)
        : m_dPtX_m(other.m_dPtX_m), m_dPtY_m(other.m_dPtY_m) {}
    ~SLanePoint() {}
};

struct SLane {
    unsigned int m_nLaneID;
    std::vector<SLanePoint> m_vecLanePoint;

public:
    SLane() 
        : m_nLaneID(0) { m_vecLanePoint.clear(); }
    SLane(unsigned int nLaneID, std::vector<SLanePoint> vecLanePoint)
        : m_nLaneID(nLaneID), m_vecLanePoint(vecLanePoint) {}
    SLane(const SLane &other)
        : m_nLaneID(other.m_nLaneID), m_vecLanePoint(other.m_vecLanePoint) {}
    ~SLane() {}
};

struct SLanes {
    std::vector<SLane> m_vecLanes;

public:
    SLanes() { m_vecLanes.clear(); }
    ~SLanes() {}

public:
    bool ImportLaneCsvFile(std::string strImportLaneCsvFilePath);
    bool ExportLaneCsvFile(std::string strExportLaneCsvFilePath);

private:
    // Internal functions for file management
    const static int STRING_PARSING_MAX_SIZE = 1000;
    std::string TimeToString(void);
    size_t CountFileLines(std::istream &is);
    bool CheckPathExistence(std::string strFilePath);
    std::vector<std::string> FindFileList(std::string strPath, std::string strExtension);
    std::vector<std::string> SplitString(std::string strOrigin, std::string strTok);

    // Internal functions for debug information
    void DisplayInfo(std::string str);
    void DisplayError(std::string str);
    void DisplayProgress(std::string str, unsigned int nNumOfProgress, unsigned int nNumOfTotal);
};

namespace interface{
    typedef struct {
        double x{0.0};
        double y{0.0};
    } Point2D;

    typedef struct {
        std::string frame_id;
        std::string id;
        std::vector<Point2D> point;
    } Lane;

    typedef struct {
        std::string frame_id;
        std::string id;
        std::vector<Lane> lane;
    } Lanes;

    typedef struct {
        std::string frame_id;
        std::string id;
        double a0{0.0};
        double a1{0.0};
        double a2{0.0};
        double a3{0.0};
    } PolyfitLane;

    typedef struct {
        std::string frame_id;        
        std::vector<PolyfitLane> polyfitlanes;
    } PolyfitLanes;
} // namespace interface