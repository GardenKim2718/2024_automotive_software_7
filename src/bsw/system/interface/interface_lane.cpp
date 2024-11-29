/**
 * @copyright Hanyang University, Department of Automotive Engineering, 2024. All rights reserved. 
 *            Subject to limited distribution and restricted disclosure only.
 *            
 * @file      interface_lane.cpp
 * @brief     Lane structure, import and export
 * 
 * @date      2018-11-20 created by Kichun Jo (kichunjo@hanyang.ac.kr)
 *            2023-08-07 updated by Yuseung Na (yuseungna@hanyang.ac.kr)
 *              : make as library file
 */

#include "interface_lane.hpp"

bool SLanes::ImportLaneCsvFile(std::string strImportLaneCsvFilePath) {
    DisplayInfo("Import Name is " + strImportLaneCsvFilePath);
    std::vector<std::string> vecStrLaneCsvFiles =
        FindFileList(strImportLaneCsvFilePath, ".csv");
    if (vecStrLaneCsvFiles.size() <= 0) {
        DisplayError("No available Lane Csv files");
        return false;
    }

    // Clear the Lanes vector and Id
    m_vecLanes.clear();
    int nLaneID = 0;

    for (auto vecIt = vecStrLaneCsvFiles.begin();
        vecIt != vecStrLaneCsvFiles.end(); vecIt++) {
        // Lane file path
        std::string strLaneFile = strImportLaneCsvFilePath + "/" + *vecIt + ".csv";

        // One Lane structure and ID
        SLane tmpLane;

        std::vector<std::string> vecstrSplitLine = SplitString(*vecIt, "_");
        tmpLane.m_nLaneID = (unsigned int)(stoi(vecstrSplitLine[1]));

        // File read
        std::ifstream ifsFile(strLaneFile);
        unsigned int nTotalNumIter = (unsigned int)CountFileLines(ifsFile); // Get number of lines
        bool bFlagFirstLine = true;
        unsigned int nNumIter = 0;

        while (!ifsFile.eof()) {

            // Progress display
            DisplayProgress("Parse Lane Csv File", nNumIter, nTotalNumIter - 1);
            nNumIter++;

            // Get line information
            char inputString[STRING_PARSING_MAX_SIZE];
            ifsFile.getline(inputString, STRING_PARSING_MAX_SIZE);

            // First line check
            if (bFlagFirstLine == true) {
                bFlagFirstLine = false;
                continue;
            }

            // Empty line check
            std::string strTmpLine = std::string(inputString);
            if (strTmpLine.compare("") == 0)
                continue;

            // Get data
            std::vector<std::string> vecstrSplitLine = SplitString(strTmpLine, ",");

            // Parse
            tmpLane.m_vecLanePoint.push_back(SLanePoint((double)stof(vecstrSplitLine[0]), (double)stof(vecstrSplitLine[1])));
        }
        // Push back to m_vecLanes
        m_vecLanes.push_back(tmpLane);

        // Close file
        ifsFile.close();
    }

    // return
    return true;
}

bool SLanes::ExportLaneCsvFile(std::string strExportLaneCsvFilePath) {
    // Generate path
    if (CheckPathExistence(strExportLaneCsvFilePath) != true) {
        DisplayError("ExportLaneCsvFile Failed - File path cannot find!");
        return false;
    }

    for (auto vecLanesIt = m_vecLanes.begin();
        vecLanesIt != m_vecLanes.end(); vecLanesIt++) {
        SLane &tmpLane = *vecLanesIt;

        // Configuration
        std::string strRecordingFile =
            strExportLaneCsvFilePath + "/" + "Lane_" +
            std::to_string(tmpLane.m_nLaneID) + ".csv";
        std::ofstream file_out;
        file_out.open(strRecordingFile.c_str());
        file_out.precision(12);

        // File header
        file_out << "LanePointX,"
                << "LanePointY"
                << "\n";

        for (auto vecLaneIt = tmpLane.m_vecLanePoint.begin();
            vecLaneIt != tmpLane.m_vecLanePoint.end(); vecLaneIt++) {
            SLanePoint &tmpLanePoint = *vecLaneIt;

            file_out << std::to_string(tmpLanePoint.m_dPtX_m) << ","
                    << std::to_string(tmpLanePoint.m_dPtY_m) << "\n";
        }

        // Export
        file_out.close();
        DisplayInfo("Export complete: " + strRecordingFile);
    }
    return true;
}

std::string SLanes::TimeToString(void) {
    struct tm *t;
    time_t timer;

    timer = time(NULL);
    t = localtime(&timer);

    std::stringstream ssTmp;
    ssTmp << std::setw(2) << std::setfill('0') << t->tm_year + 1900;
    ssTmp << std::setw(2) << std::setfill('0') << t->tm_mon + 1;
    ssTmp << std::setw(2) << std::setfill('0') << t->tm_mday;
    ssTmp << "_";
    ssTmp << std::setw(2) << std::setfill('0') << t->tm_hour;
    ssTmp << std::setw(2) << std::setfill('0') << t->tm_min;
    ssTmp << std::setw(2) << std::setfill('0') << t->tm_sec;
    return ssTmp.str();
}

size_t SLanes::CountFileLines(std::istream &is) {
    // skip when bad
    if (is.bad())
        return 0;
    // save state
    std::istream::iostate state_backup = is.rdstate();
    // clear state
    is.clear();
    std::streampos pos_backup = is.tellg();

    is.seekg(0);
    size_t line_cnt;
    size_t lf_cnt = std::count(std::istreambuf_iterator<char>(is),
                                std::istreambuf_iterator<char>(), '\n');
    line_cnt = lf_cnt;
    // if the file is not end with '\n' , then line_cnt should plus 1
    is.unget();
    if (is.get() != '\n') {
        ++line_cnt;
    }

    // recover state
    is.clear(); // previous reading may set eofbit
    is.seekg(pos_backup);
    is.setstate(state_backup);

    return line_cnt;
}

bool SLanes::CheckPathExistence(std::string strFilePath) {
    // Create a Path object from given path string
    boost::filesystem::path pathObj(strFilePath);
    // Check if path exists and is of a directory file
    if (boost::filesystem::exists(pathObj) &&
        boost::filesystem::is_directory(pathObj))
        return true;
    else {
        if (boost::filesystem::create_directories(strFilePath.c_str()) != true) {
            return false;
        }
    }

    return true;
}

std::vector<std::string> SLanes::FindFileList(std::string strPath, std::string strExtension) {
    // Define output vector
    std::vector<std::string> vecFileList;
    vecFileList.clear();

    if (!strPath.empty()) {
        // Set path
        boost::filesystem::path apk_path(strPath);

        // Set iterator
        boost::filesystem::recursive_directory_iterator end;

        // iteration
        for (boost::filesystem::recursive_directory_iterator i(apk_path); i != end;
            ++i) {
            const boost::filesystem::path cp = (*i);
            // Get file name with extension
            std::string strFileName = cp.filename().string();
            std::string strFileExt = boost::filesystem::extension(strFileName);

            // Check extension
            if (std::string(strFileExt).compare(strExtension) != 0)
                continue;

            // Get file name
            vecFileList.push_back(cp.stem().string());
        }
    }
    return vecFileList;
}

std::vector<std::string> SLanes::SplitString(std::string strOrigin, std::string strTok) {
    size_t cutAt;
    std::vector<std::string> vecstrSplitInputLine;
    while ((cutAt = strOrigin.find_first_of(strTok)) != strOrigin.npos) {
        if (cutAt > 0) {
            vecstrSplitInputLine.push_back(strOrigin.substr(0, cutAt));
        }

        strOrigin = strOrigin.substr(cutAt + 1);
    }

    if (strOrigin.length() > 0) {
        vecstrSplitInputLine.push_back(strOrigin.substr(0, cutAt));
    }
    return vecstrSplitInputLine;
}

void SLanes::DisplayInfo(std::string str) { 
    std::cout << str << std::endl; 
}
void SLanes::DisplayError(std::string str) {
    std::cerr << str << std::endl;
}
void SLanes::DisplayProgress(std::string str, unsigned int nNumOfProgress,
                                 unsigned int nNumOfTotal) {
    if (nNumOfTotal == 0) {
        DisplayError("Cannot display progress");
    }

    static int prev_progress = 0;
    int pres_progress =
        (int)((double)nNumOfProgress / (double)nNumOfTotal * 100.);

    if (prev_progress != pres_progress) {
        std::cout << str << ": ";
        prev_progress = pres_progress;
        float progress = (float)pres_progress / 100.f;
        int barWidth = 30;
        std::cout << "[";
        int pos = barWidth * progress;
        for (int i = 0; i < barWidth; ++i) {
            if (i < pos)
                std::cout << "=";
            else if (i == pos)
                std::cout << ">";
            else
                std::cout << " ";
        }
        std::cout << "] " << int(progress * 100.0) << " %\r";
        std::cout.flush();
    }

    if (nNumOfProgress == nNumOfTotal) {
        std::cout << std::endl;
    }
}