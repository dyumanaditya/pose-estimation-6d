/**
 * @file csv_parser.cpp
 * @authors Arjun Datta (arjunvirdatta@gmail.com), Dyuman Aditya (dyuman.aditya@gmail.com)
 * @brief Source file for parsing csv with poses
 * @version 1.0
 * @date 2021-07-10
 * 
 * @copyright Copyright (c) 2021 (we need licence)
 * 
 */


#include "csv_parser.h"

#include <Eigen/Geometry>
#include <iostream>



CSVParser::CSVParser(std::string csv_path, std::string csv_name) : csv(csv_path + "/" + csv_name + ".csv")
{
}


CSVParser::~CSVParser()
{
}


void CSVParser::writePoseToCsv(Eigen::Matrix4f transformation)
{
    Eigen::Affine3f t;
    Eigen::Quaternionf quaternion;
    Eigen::Vector3f translation;
    t.matrix() = transformation;

    quaternion = t.rotation();
    translation = t.translation();

    

    csv << translation[0] << "," << translation[1] << "," << translation[2] << ",";
    csv << quaternion.x() << "," << quaternion.y() << "," << quaternion.z() << "," << quaternion.w() << "\n";
    csv.flush();
}



